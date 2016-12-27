//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "zniimaqdxgrabcamera_p.h"

#include <QDebug>
#include <QTime>
#include <QTimer>
#include <QThread>

#include "stdint.h"

/// Constants
const char* DefaultAttributeRoot = "CameraAttributes";
const IMAQdxAttributeVisibility DefaultAttributeVisibility = IMAQdxAttributeVisibilityAdvanced;  //IMAQdxAttributeVisibilitySimple;


QString getNIVisionErrorDescription(int errorCode)
{
    Int8* errorText = imaqGetErrorText(errorCode);
    Int8 dialogText[512];
    sprintf_s(dialogText, "NI-Vision Error 0x%08X: %s", errorCode, errorText);

    qWarning() << dialogText;

    imaqDispose(errorText);

    return QString(dialogText);
}

QString getNIIMAQdxErrorDescription(IMAQdxError errorCode)
{
    Int8 errorText[512];
    sprintf_s(errorText, "NI-IMAQdx Error 0x%08X: ", errorCode);
    IMAQdxGetErrorString (errorCode, errorText + strlen(errorText), sizeof(errorText) - strlen(errorText));

    qWarning() << errorText;

    return QString(errorText);
}

namespace Z3D
{

/**
 * typedef     void (NI_FUNC *AttributeUpdatedEventCallbackPtr)(IMAQdxSession id, const char* name, void* callbackData);
 */
void __stdcall attributeUpdatedEventCallback(IMAQdxSession id, const char* name, void* callbackData)
{
    reinterpret_cast<ZNIIMAQdxGrabCameraPrivate*>(callbackData)->attributeUpdatedEventCallbackInternal(id, name, callbackData);
}

ZNIIMAQdxGrabCameraPrivate *ZNIIMAQdxGrabCameraPrivate::getCameraByName(QString name)
{
    SESSION_ID session;

    IMAQdxError status = IMAQdxOpenCamera(qPrintable(name), IMAQdxCameraControlModeController, &session);

    if (status != IMAQdxErrorSuccess) {
        qWarning() << getNIIMAQdxErrorDescription(status);
        return 0;
    }

    return new ZNIIMAQdxGrabCameraPrivate(session);
}

ZNIIMAQdxGrabCameraPrivate::ZNIIMAQdxGrabCameraPrivate(SESSION_ID sessionId, QObject *parent) :
    QObject(parent),
    m_session(sessionId),
    m_imaqImage(0),
    m_imaqBufferSize(20),
    m_acquisitionCounter(0),
    m_stopThreadRequested(true)
{
    /// obtain properties to generate unique camera id
    char buffer[IMAQDX_MAX_API_STRING_LENGTH] = { 0 };
    /*
  IMAQdxGetAttribute(m_session, "DeviceModelName", IMAQdxValueTypeString, buffer);
  m_uuid = QString("NIIMAQdx-%1").arg(buffer);

  IMAQdxGetAttribute(m_session, "DeviceID", IMAQdxValueTypeString, buffer);
  m_uuid += QString("-%1").arg(buffer);
*/
    IMAQdxGetAttribute(m_session, "DeviceUserID", IMAQdxValueTypeString, buffer);
    m_uuid += QString("ZNIIMAQdxGrab-%1").arg(buffer);

    /// do we need this?
    //imaqSetWindowThreadPolicy(IMAQ_SEPARATE_THREAD);

    /// create image
    m_imaqImage = imaqCreateImage(IMAQ_IMAGE_U8, 0);
    if (!m_imaqImage) {
        emit error( getNIVisionErrorDescription(imaqGetLastError()) );
    }

    /// create a thread for the camera and move the camera to it
    QThread *cameraThread = new QThread();
    qDebug() << qPrintable(
                    QString("[%1] moving camera to its own thread (0x%2)")
                    .arg(m_uuid)
                    .arg((long)cameraThread, 0, 16));
    this->moveToThread(cameraThread);
    /// start thread with the highest priority
    cameraThread->start(QThread::TimeCriticalPriority);

    qDebug() << "created camera:" << m_uuid;
}

ZNIIMAQdxGrabCameraPrivate::~ZNIIMAQdxGrabCameraPrivate()
{
    qDebug() << "destroying camera:" << m_uuid;

    /// close the session.
    IMAQdxCloseCamera(m_session);

    /// dispose of the image.
    if (m_imaqImage)
        imaqDispose(m_imaqImage);
}

bool ZNIIMAQdxGrabCameraPrivate::startAcquisition()
{
    qDebug() << Q_FUNC_INFO;

    m_currentAcquisitionCounter = m_acquisitionCounter;
    m_lastReturnedBufferNumber = 0;

    IMAQdxError status = IMAQdxErrorSuccess;

    try {
        /// configure _continuous_ acquisition with a buffer of size m_imaqBufferSize
        status = IMAQdxConfigureAcquisition(m_session, TRUE, m_imaqBufferSize);
        if (status)
            throw status;
/*
 * THIS VERSION WORKS WITH GRAB LOOP, NOT CALLBACK
        /// register asynchronous callback to be called every 1 frame
        status = IMAQdxRegisterFrameDoneEvent(m_session, 1, frameDoneCallback, this);
        if (status)
            throw status;
*/
        status = IMAQdxStartAcquisition(m_session);
        if (status)
            throw status;
    }
    catch (IMAQdxError err) {
        emit error( getNIIMAQdxErrorDescription(err) );
    }

    if (status != IMAQdxErrorSuccess) {
        /// something failed, stop acquisition
        q_ptr->stopAcquisition();
        return false;
    }

    /// start running grab loop in the objects thread
    m_stopThreadRequested = false;
    QTimer::singleShot(0, this, SLOT(grabLoop()));

    return true;
}

bool ZNIIMAQdxGrabCameraPrivate::stopAcquisition()
{
    m_stopThreadRequested = true;

    qDebug() << Q_FUNC_INFO;
    qDebug() << "last returned buffer number:" << m_lastReturnedBufferNumber;

    IMAQdxStopAcquisition(m_session);
    IMAQdxUnconfigureAcquisition(m_session);

    m_acquisitionCounter++;
    if (m_acquisitionCounter > INT_MAX/2)
        m_acquisitionCounter = 0;

    return true;
}

QList<Z3D::ZCameraInterface::ZCameraAttribute> ZNIIMAQdxGrabCameraPrivate::getAllAttributes()
{
    qDebug() << Q_FUNC_INFO;

    IMAQdxError status = IMAQdxErrorSuccess;

    uInt32 attributeCount = 0;
    IMAQdxAttributeInformation* attributeArray = NULL;

    QList<Z3D::ZCameraInterface::ZCameraAttribute> setts;

    status = IMAQdxEnumerateAttributes2(m_session, NULL, &attributeCount, DefaultAttributeRoot, DefaultAttributeVisibility);
    if (status) {
        QString errorStr = getNIIMAQdxErrorDescription(status);
        qWarning() << errorStr;
        emit error(errorStr);
        return setts;
    }

    attributeArray = new IMAQdxAttributeInformation[attributeCount];
    if (!attributeArray)
        return setts;

    status = IMAQdxEnumerateAttributes2(m_session, attributeArray, &attributeCount, DefaultAttributeRoot, DefaultAttributeVisibility);
    if (status) {
        QString errorStr = getNIIMAQdxErrorDescription(status);
        qWarning() << errorStr;
        emit error(errorStr);
        return setts;
    }

    for (uInt32 i = 0; i < attributeCount; ++i) {
        QString attrName(attributeArray[i].Name);

        const char* attrNameC = qPrintable(attrName);

        Z3D::ZCameraInterface::ZCameraAttribute *actualSetting = nullptr;

        if (!m_attributes.contains(attrName)) {
            /// add new
            actualSetting = new Z3D::ZCameraInterface::ZCameraAttribute();

            /// register callback to notify that an attribute has changed
            IMAQdxRegisterAttributeUpdatedEvent(m_session, attrNameC, attributeUpdatedEventCallback, this);

            actualSetting->name = attrName;

            /// obtain attribute description
            char description[IMAQDX_MAX_API_STRING_LENGTH];
            IMAQdxGetAttributeDescription(m_session, attrNameC, description, IMAQDX_MAX_API_STRING_LENGTH);
            actualSetting->description = QString(description);

            switch (attributeArray[i].Type) {
            case IMAQdxAttributeTypeU32:
                actualSetting->type = Z3D::ZCameraInterface::CameraAttributeTypeInt;
                break;
            case IMAQdxAttributeTypeI64:
                actualSetting->type = Z3D::ZCameraInterface::CameraAttributeTypeLong;
                break;
            case IMAQdxAttributeTypeF64:
                actualSetting->type = Z3D::ZCameraInterface::CameraAttributeTypeFloat;
                break;
            case IMAQdxAttributeTypeString:
                actualSetting->type = Z3D::ZCameraInterface::CameraAttributeTypeString;
                break;
            case IMAQdxAttributeTypeEnum:
                actualSetting->type = Z3D::ZCameraInterface::CameraAttributeTypeEnum;
                break;
            case IMAQdxAttributeTypeBool:
                actualSetting->type = Z3D::ZCameraInterface::CameraAttributeTypeBool;
                break;
            case IMAQdxAttributeTypeCommand:
                actualSetting->type = Z3D::ZCameraInterface::CameraAttributeTypeCommand;
                break;
            case IMAQdxAttributeTypeBlob:
                qWarning() << "we don't use the type BLOB for now...";
                break;
            default:
                qWarning() << actualSetting->name << ": type is unknown, are we missing this one?" << attributeArray[i].Type;
            }

            /// add to internal map
            m_attributes[attrName] = actualSetting;
        } else {
            /// get from internal map
            actualSetting = m_attributes[attrName];
        }

        actualSetting->readable = attributeArray[i].Readable;
        actualSetting->writable = attributeArray[i].Writable;

        /// get current value
        if (actualSetting->readable)
            actualSetting->value = getAttribute(actualSetting->name);

        switch (attributeArray[i].Type) {
        case IMAQdxAttributeTypeU32:
        case IMAQdxAttributeTypeI64:
        case IMAQdxAttributeTypeF64:
            IMAQdxGetAttributeMaximum(m_session, attrNameC, IMAQdxValueTypeF64, &actualSetting->maximumValue);
            IMAQdxGetAttributeMinimum(m_session, attrNameC, IMAQdxValueTypeF64, &actualSetting->minimumValue);
            break;
        case IMAQdxAttributeTypeString:
            break;
        case IMAQdxAttributeTypeEnum:
            actualSetting->enumNames.clear();
            uInt32 enumSize;
            IMAQdxEnumerateAttributeValues(m_session, attrNameC, 0, &enumSize);
            if (enumSize > 0) {
                IMAQdxEnumItem *enumList = new IMAQdxEnumItem[enumSize];
                IMAQdxEnumerateAttributeValues(m_session, attrNameC, enumList, &enumSize);

                for (unsigned int idx=0; idx<enumSize; ++idx) {
                    QString thisEnumName(enumList[idx].Name);
                    actualSetting->enumNames << thisEnumName;
                    if (actualSetting->value.toString() == thisEnumName)
                        actualSetting->enumValue = idx;
                }

                delete[] enumList;
            }
            break;
        case IMAQdxAttributeTypeBool:
            break;
        case IMAQdxAttributeTypeCommand:
            break;
        case IMAQdxAttributeTypeBlob:
            qWarning() << "we don't use the type BLOB for now...";
            break;
        default:
            qWarning() << actualSetting->name << ": type is unknown, are we missing this one?" << attributeArray[i].Type;
        }

        setts.push_back( *actualSetting );
    }

    /// delete attribute array
    delete [] attributeArray;

    return setts;
}

QVariant ZNIIMAQdxGrabCameraPrivate::getAttribute(const QString &name) const
{
    QVariant value;

    IMAQdxAttributeType type;
    IMAQdxError status = IMAQdxGetAttributeType(m_session, qPrintable(name), &type);

    if (status != IMAQdxErrorSuccess) {
        qWarning() << QString("unable to get type for attribute '%1'").arg(name);
        return value;
    }

    switch (type) {
    case IMAQdxAttributeTypeU32:
        uint32_t uint32Value;
        IMAQdxGetAttribute(m_session, qPrintable(name), IMAQdxValueTypeU32, &uint32Value);
        value = uint32Value;
//        qDebug() << name << ": type is U32" << ":" << value;
        break;
    case IMAQdxAttributeTypeI64:
        int64_t int64Value;
        IMAQdxGetAttribute(m_session, qPrintable(name), IMAQdxValueTypeI64, &int64Value);
        value = int64Value;
//        qDebug() << name << ": type is I64" << ":" << value;
        break;
    case IMAQdxAttributeTypeF64:
        float64 float64Value;
        IMAQdxGetAttribute(m_session, qPrintable(name), IMAQdxValueTypeF64, &float64Value);
        value = float64Value;
//        qDebug() << name << ": type is F64" << ":" << value;
        break;
    case IMAQdxAttributeTypeString:
    {
        char buffer[IMAQDX_MAX_API_STRING_LENGTH] = { 0 };
        IMAQdxGetAttribute(m_session, qPrintable(name), IMAQdxValueTypeString, buffer);
        value = QString(buffer);
//        qDebug() << name << ": type is String" << ":" << value;
    }
        break;
    case IMAQdxAttributeTypeEnum:
    {
        char buffer[IMAQDX_MAX_API_STRING_LENGTH] = { 0 };
        IMAQdxGetAttribute(m_session, qPrintable(name), IMAQdxValueTypeString, buffer);
        value = QString(buffer);
//        qDebug() << name << ": type is ENUM" << ":" << value;
    }
        break;
    case IMAQdxAttributeTypeBool:
        bool boolValue;
        IMAQdxGetAttribute(m_session, qPrintable(name), IMAQdxValueTypeBool, &boolValue);
        value = boolValue;
//        qDebug() << name << ": type is bool" << ":" << value;
        break;
    case IMAQdxAttributeTypeCommand:
        qWarning() << "we don't use the type COMMAND for now...";
        break;
    case IMAQdxAttributeTypeBlob:
        qWarning() << "we don't use the type BLOB for now...";
        break;
    default:
        qWarning() << name << ": type is unknown, are we missing this one?" << type;
    }

    return value;
}

bool ZNIIMAQdxGrabCameraPrivate::setBufferSize(int bufferSize)
{
    m_imaqBufferSize = bufferSize;
    return true;
}

bool ZNIIMAQdxGrabCameraPrivate::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    //    qDebug() << "setting attribute" << name << ":" << value;

    IMAQdxAttributeType type;
    IMAQdxError status = IMAQdxGetAttributeType(m_session, qPrintable(name), &type);

    if (status != IMAQdxErrorSuccess) {
        qWarning() << QString("unable to get type for attribute '%1'").arg(name);
        return false;
    }

    switch (type) {
    case IMAQdxAttributeTypeU32:
    {
        uint32_t uint32Value = value.toUInt();
        //        qDebug() << QString("setting attribute '%1' (type '%2') to '%3'").arg(name).arg("U32").arg(uint32Value);
        status = IMAQdxSetAttribute(m_session, qPrintable(name), IMAQdxValueTypeU32, uint32Value);
    }
        break;
    case IMAQdxAttributeTypeI64:
    {
        int64_t int64Value = value.toLongLong();
        //        qDebug() << QString("setting attribute '%1' (type '%2') to '%3'").arg(name).arg("I64").arg(int64Value);
        status = IMAQdxSetAttribute(m_session, qPrintable(name), IMAQdxValueTypeI64, int64Value);
    }
        break;
    case IMAQdxAttributeTypeF64:
    {
        float64 float64Value = value.toFloat();
        //        qDebug() << QString("setting attribute '%1' (type '%2') to '%3'").arg(name).arg("F64").arg(float64Value);
        status = IMAQdxSetAttribute(m_session, qPrintable(name), IMAQdxValueTypeF64, float64Value);
    }
        break;
    case IMAQdxAttributeTypeString:
        //        qDebug() << QString("setting attribute '%1' (type '%2') to '%3'").arg(name).arg("string").arg(value.toString());
        status = IMAQdxSetAttribute(m_session, qPrintable(name), IMAQdxValueTypeString, qPrintable(value.toString()));
        break;
    case IMAQdxAttributeTypeEnum:
        //        qDebug() << QString("setting attribute '%1' (type '%2') to '%3'").arg(name).arg("ENUM").arg(value.toString());
        status = IMAQdxSetAttribute(m_session, qPrintable(name), IMAQdxValueTypeString, qPrintable(value.toString()));
        break;
    case IMAQdxAttributeTypeBool:
    {
        bool boolValue = value.toBool();
        //        qDebug() << QString("setting attribute '%1' (type '%2') to '%3'").arg(name).arg("bool").arg(boolValue);
        status = IMAQdxSetAttribute(m_session, qPrintable(name), IMAQdxValueTypeBool, boolValue);
    }
        break;
    case IMAQdxAttributeTypeCommand:
        status = IMAQdxSetAttribute(m_session, qPrintable(name), IMAQdxValueTypeBool, true);
        //qWarning() << "we don't use the type COMMAND for now...";
        break;
    case IMAQdxAttributeTypeBlob:
        qWarning() << "we don't use the type BLOB for now...";
        break;
    default:
        qWarning() << name << ": type is unknown, are we missing this one?" << type;
    }

    /// debug
    if (status != IMAQdxErrorSuccess) {
        qWarning() << "unable to set attribute" << name << "to" << value;
        return false;
    }

    if (notify) {
        /// update properties widget
        emit attributeChanged(name, value);
    }

    /// return true, status == success
    return true;
}

void ZNIIMAQdxGrabCameraPrivate::attributeUpdatedEventCallbackInternal(IMAQdxSession id, const char *name, void *callbackData)
{
    qDebug() << name;

    QString attrName(name);
    emit attributeChanged(attrName, getAttribute(attrName));
}

//uInt32 ZNIIMAQdxGrabCameraPrivate::frameDoneCallback(IMAQdxSession cbsession, uInt32 bufferNumber, void *callbackData)
//{
//    return reinterpret_cast<ZNIIMAQdxGrabCameraPrivate*>(callbackData)->frameDoneCallbackInternal(cbsession, bufferNumber);
//}

//uInt32 ZNIIMAQdxGrabCameraPrivate::frameDoneCallbackInternal(IMAQdxSession cbSession, uInt32 bufferNumber)
//{
//    /// check if the callback is valid (ie: corresponds to the current acquisition)
//    if (m_currentAcquisitionCounter != m_acquisitionCounter)
//        return true;

//    /// this is kind of useless if we check the validity of the callback
//    if (!q_ptr->isRunning())
//        return true;

//    if (cbSession != m_session) {
//        qWarning() << "sessions differ! this session:" << m_session << ", callback session:" << cbSession;
//        return true;
//    }

//    //QTime processingTime;
//    //processingTime.start();

//    IMAQdxError status = IMAQdxErrorSuccess;

//    uInt32 returnedBufferNumber;

//    /// acquire image
//    status = IMAQdxGetImage(m_session, m_imaqImage, IMAQdxBufferNumberModeBufferNumber, bufferNumber, &returnedBufferNumber);
//    //status = IMAQdxGetImage(m_session, m_imaqImage, IMAQdxBufferNumberModeLast, bufferNumber, &returnedBufferNumber);

//    /// something went wrong while trying to get the image
//    if (status) {
//        QString errorStr = getNIIMAQdxErrorDescription(status);
//        qWarning() << q_ptr->uuid() << errorStr;
//        emit error( errorStr );
//        return true;
//    }

//    if (bufferNumber != returnedBufferNumber) {
//        qCritical() << q_ptr->uuid() << " missed the frame! requested:" << bufferNumber << "returned:" << returnedBufferNumber;
//    }

//    /// check frame number
//    if (returnedBufferNumber == m_lastReturnedBufferNumber) {
//        qWarning() << q_ptr->uuid() << "skipping repeated buffer" << returnedBufferNumber;
//        return true;
//    } else /*if (bufferNumber != returnedBufferNumber) {

//    } else*/ if (returnedBufferNumber != m_lastReturnedBufferNumber + 1) {
//        qCritical() << q_ptr->uuid() << "skipped some frame/s! previous was:" << m_lastReturnedBufferNumber << "returned:" << returnedBufferNumber;
//    } else {
//        //qDebug() << "received frame" << returnedBufferNumber;
//    }
//    m_lastReturnedBufferNumber = returnedBufferNumber;

//    processImage();

//    return true;
//}

void ZNIIMAQdxGrabCameraPrivate::grabLoop()
{
    //QTime processingTime;
    //processingTime.start();

    /// start grabbing images until either the user tells us to stop or we encounter an error
    IMAQdxError status = IMAQdxErrorSuccess;
    uInt32 returnedBufferNumber;

    while (!m_stopThreadRequested) {

        //status = IMAQdxGetImage(m_session, m_imaqImage, IMAQdxBufferNumberModeNext, 0, &returnedBufferNumber);
        /// que pasa si por alguna razon demora un ratito y cuando pido de nuevo ya pasó una, por mas que no la haya perdido?
        /// vamos a probar pidiendo la siguiente en base al numero que deberia estar a continuacion
        status = IMAQdxGetImage(m_session, m_imaqImage, IMAQdxBufferNumberModeBufferNumber, m_lastReturnedBufferNumber+1, &returnedBufferNumber);

        //qDebug() << "getImageTime:" << processingTime.restart() << "msecs";

        /// something went wrong while trying to get the image
        if (status) {
            if (status == IMAQdxErrorCameraNotRunning) {
                /// if error is "acquisition not running" its probably because the acquisition was stopped
                //qDebug() << q_ptr->uuid() << "acquisition is stopping...";
                continue;
            }

            if (status == IMAQdxErrorTimeout) {
                /// if error is timeout dont bother and continue
                qDebug() << q_ptr->uuid() << "skipping timeout...";
                continue;
            }

            /// if we reached here, it's an error we don't expect
            QString errorStr = getNIIMAQdxErrorDescription(status);
            qWarning() << q_ptr->uuid() << errorStr;
            emit error( errorStr );
            break;
        }

        /// check frame number
        if (returnedBufferNumber == m_lastReturnedBufferNumber) {
            QString errorStr = QString("[%1] skipping repeated buffer %2")
                    .arg(q_ptr->uuid())
                    .arg(returnedBufferNumber);
            emit warning(errorStr);
            qWarning() << errorStr;
            continue; /// esto no debería pasar nunca...
        } else if (returnedBufferNumber != m_lastReturnedBufferNumber + 1) {
            QString errorStr = QString("[%1] lost frames between %2 and %3")
                    .arg(q_ptr->uuid())
                    .arg(m_lastReturnedBufferNumber)
                    .arg(returnedBufferNumber);
            emit error(errorStr);
            qCritical() << errorStr;
        } else {
            //qDebug() << "received frame" << returnedBufferNumber;
        }
        m_lastReturnedBufferNumber = returnedBufferNumber;

        processImage();

        //qDebug() << "processingTime:" << processingTime.restart() << "msecs";
    }

    /// stop por las dudas poner de nuevo en true
    m_stopThreadRequested = true;
}

void ZNIIMAQdxGrabCameraPrivate::processImage()
{
    ImageInfo imageInfo;
    imaqGetImageInfo(m_imaqImage, &imageInfo);

//    qDebug() << "image size:" << imageInfo.xRes << "x" << imageInfo.yRes;
//    qDebug() << "image offset x:" << imageInfo.xOffset << ", y:" << imageInfo.yOffset;
//    qDebug() << "image step x:" << imageInfo.stepX << ", y:" << imageInfo.stepY;

    int bytesPerPixel = -1;

    switch (imageInfo.imageType) {
    case IMAQ_IMAGE_U8:
        /// = 0, //The image type is 8-bit unsigned integer grayscale.
        bytesPerPixel = 1;
        break;

    case IMAQ_IMAGE_U16:
        /// = 7, //The image type is 16-bit unsigned integer grayscale.
        qWarning() << "image U16 not implemented";
        break;

    case IMAQ_IMAGE_I16:
        /// = 1, //The image type is 16-bit signed integer grayscale.
        bytesPerPixel = 2;
        break;

    case IMAQ_IMAGE_SGL:
        /// = 2, //The image type is 32-bit floating-point grayscale.
        qWarning() << "image SGL not implemented";
        break;

    case IMAQ_IMAGE_COMPLEX:
        /// = 3, //The image type is complex.
        qWarning() << "image complex not implemented";
        break;

    case IMAQ_IMAGE_RGB:
        /// = 4, //The image type is RGB color.
        qWarning() << "image RGB not implemented";
        break;

    case IMAQ_IMAGE_HSL:
        /// = 5, //The image type is HSL color.
        qWarning() << "image HSL not implemented";
        break;

    case IMAQ_IMAGE_RGB_U64:
        /// = 6, //The image type is 64-bit unsigned RGB color.
        qWarning() << "image U64 not implemented";
        break;
    }

    if (bytesPerPixel > 0) {
        /// get image from buffer
        /*ImageGrayscale::Ptr &currentImage = q_ptr->getNextBufferImage();

        /// this was used when we copied driver image buffer to own image buffer
        /// this was useless, we could increase the buffer size and we'll
        /// have the same results and reduce the data copying overhead

        /// if null or different size
        if (currentImage.isNull() ||
                currentImage->width() != imageInfo.xRes ||
                currentImage->height() != imageInfo.yRes ||
                currentImage->bytesPerPixel() != bytesPerPixel) {

            ImageGrayscale::Ptr newImage(new ImageGrayscale(imageInfo.xRes, imageInfo.yRes,
                                                            imageInfo.xOffset, imageInfo.yOffset,
                                                            bytesPerPixel));

            currentImage.swap( newImage );
        }

        /// copy data
        currentImage->setBuffer(imageInfo.imageStart);

        /// This is without copying data, use same buffer
        ImageGrayscale::Ptr newImage(new ImageGrayscale(imageInfo.xRes, imageInfo.yRes,
                                                        imageInfo.xOffset, imageInfo.yOffset,
                                                        bytesPerPixel,
                                                        imageInfo.imageStart));

        currentImage.swap( newImage );
        */

        /// This is without copying data, use same buffer
        ZImageGrayscale::Ptr currentImage = q_ptr->getNextBufferImage(imageInfo.xRes, imageInfo.yRes,
                                                                     imageInfo.xOffset, imageInfo.yOffset,
                                                                     bytesPerPixel,
                                                                     imageInfo.imageStart);

        /// set image number
        currentImage->setNumber(m_lastReturnedBufferNumber);

        /// notify
        emit newImageReceived(currentImage);
    }


//    /// for fps calculation
//    static uInt32 lastBufferNumber = -1;
//    static QTime timeCounter;
//    if (lastBufferNumber < 0) {
//        timeCounter.start();
//    }

//    /// every once in a while, calculate the frame rate
//    const int frameRateInterval = 500; //msecs
//    int elapsedTime = timeCounter.elapsed();
//    if (elapsedTime > frameRateInterval) {
//        float frameRate = ((float)(bufferNumber - lastBufferNumber) / (float)elapsedTime) * 1000;
//        timeCounter.restart();
//        lastBufferNumber = bufferNumber;

//        //qDebug() << "FPS:" << frameRate;
//        emit frameRateChanged(frameRate);
    //    }
}

void ZNIIMAQdxGrabCameraPrivate::saveCameraAttributes() const
{
    IMAQdxError status = IMAQdxErrorSuccess;

    uInt32 attributeCount = 0;
    IMAQdxAttributeInformation* attributeArray = NULL;
    status = IMAQdxEnumerateAttributes2(m_session, NULL, &attributeCount, DefaultAttributeRoot, DefaultAttributeVisibility);
    if (status)
        return;
    attributeArray = new IMAQdxAttributeInformation[attributeCount];
    if (!attributeArray)
        return;
    status = IMAQdxEnumerateAttributes2(m_session, attributeArray, &attributeCount, DefaultAttributeRoot, DefaultAttributeVisibility);
    if (status)
        return;

    QList< Z3D::ZCameraInterface::ZCameraAttribute > setts;

    for (uInt32 i = 0; i < attributeCount; ++i) {
        QString attributeName(attributeArray[i].Name);

        /// query read/write access
        bool32 readable = false,
               writable = false;
        IMAQdxIsAttributeReadable(m_session, qPrintable(attributeName), &readable);
        IMAQdxIsAttributeWritable(m_session, qPrintable(attributeName), &writable);

        if (readable) {
            /// get current value
            QVariant value = getAttribute(attributeName);

            if (writable) {
                /// we only care about the things we could modify
                Z3D::ZCameraInterface::ZCameraAttribute actualSetting;
                actualSetting.name = attributeName;
                actualSetting.value = value;
                setts.push_back( actualSetting );
            }
        }
    }

    /*QSettings settings(
                mUUID + QString(".config.ini"),
                QSettings::IniFormat);

    settings.beginGroup("config");
    for (int i = 0; i < setts.size(); ++i) {
        settings.beginGroup(QString("%1").arg(i, 4, 10, QLatin1Char('0')));
        settings.setValue("name", setts.at(i).name);
        settings.setValue("value", setts.at(i).value);
        settings.endGroup();
    }
    settings.endGroup();

    settings.sync();

    foreach (QString key, settings.allKeys()) {
        qDebug() << key << settings.value(key);
    }*/

    /// delete attribute array
    delete [] attributeArray;
}

} // namespace Z3D
