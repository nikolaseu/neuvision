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

#include "zavtvimbacamera.h"

#include <QDebug>
#include <QThread>

//#include "VimbaCPP/Include/Camera.h"
//#include <sstream>

namespace Z3D
{

FrameObserver::FrameObserver(AVT::VmbAPI::CameraPtr pCamera) :
    AVT::VmbAPI::IFrameObserver( pCamera )
{
    qDebug() << Q_FUNC_INFO;
}

FrameObserver::~FrameObserver()
{
    qDebug() << Q_FUNC_INFO;
}

void FrameObserver::FrameReceived( const AVT::VmbAPI::FramePtr pFrame )
{
    bool bQueueDirectly = true;
    VmbFrameStatusType eReceiveStatus;

    if ( VmbErrorSuccess == pFrame->GetReceiveStatus( eReceiveStatus )) {
        //while (eReceiveStatus != VmbFrameStatusComplete && VmbErrorSuccess == pFrame->GetReceiveStatus( eReceiveStatus )) { ; }

        /// Lock the frame queue
        m_FramesMutex.lock();
        /// Add frame to queue
        m_Frames.push( pFrame );
        /// Unlock frame queue
        m_FramesMutex.unlock();
        /// Emit the frame received signal
        emit FrameReceivedSignal( eReceiveStatus );
        bQueueDirectly = false;
    }

    /// If any error occurred we queue the frame without notification
    if (true == bQueueDirectly) {
        m_pCamera->QueueFrame( pFrame );
    }
}

// Returns the oldest frame that has not been picked up yet
AVT::VmbAPI::FramePtr FrameObserver::GetFrame()
{
    /// Lock the frame queue
    m_FramesMutex.lock();
    /// Pop frame from queue
    AVT::VmbAPI::FramePtr res = m_Frames.front();
    /// Unlock frame queue
    m_FramesMutex.unlock();
    return res;
}

void FrameObserver::ClearFrameQueue()
{
    /// Lock the frame queue
    m_FramesMutex.lock();
    /// Clear the frame queue and release the memory
    std::queue<AVT::VmbAPI::FramePtr> empty;
    std::swap( m_Frames, empty );
    /// Unlock the frame queue
    m_FramesMutex.unlock();
}






AVTVimbaCamera::AVTVimbaCamera(AVT::VmbAPI::CameraPtr cameraPtr, QObject *parent) :
    ZCameraBase(parent),
    m_cameraHandle(cameraPtr),
    m_vimbaBufferSize(bufferSize()), /// use same buffer size that ZCameraBase
    m_frameObserverInternal(0),
    m_opened(false)
{
    std::string strInfo;
    std::stringstream strInfos;

    /// If for any reason we cannot get the ID of a camera we skip it
    if (VmbErrorSuccess == m_cameraHandle->GetID( strInfo )) {
        strInfos << strInfo;
        m_deviceID = QString(strInfo.c_str());
    }

    if (VmbErrorSuccess == m_cameraHandle->GetName( strInfo )) {
        strInfos << "-" << strInfo;
    } else {
        strInfos << " [NoName]";
    }

    m_uuid = QString("ZAVTVIMBA-%1").arg( strInfos.str().c_str() );
/*
    /// set and obtain initial config
    VmbErrorType res = m_cameraHandle->Open(VmbAccessModeFull);
    if (VmbErrorSuccess == res) {
        /// Set the GeV packet size to the highest possible value
        /// (In this example we do not test whether this cam actually is a GigE cam)
        AVT::VmbAPI::FeaturePtr pCommandFeature;
        if (VmbErrorSuccess == m_cameraHandle->GetFeatureByName( "GVSPAdjustPacketSize", pCommandFeature)) {
            if (VmbErrorSuccess == pCommandFeature->RunCommand()) {
                bool bIsCommandDone = false;
                do {
                    if (VmbErrorSuccess != pCommandFeature->IsCommandDone( bIsCommandDone )) {
                        break;
                    }
                } while (false == bIsCommandDone);
            }
        }

        AVT::VmbAPI::FeaturePtr pFormatFeature;

        /// Get max image size
        res = m_cameraHandle->GetFeatureByName("WidthMax", pFormatFeature);
        if (VmbErrorSuccess == res) {
            res = pFormatFeature->GetValue(m_widthMax);
        } else {
            m_widthMax = -1;
        }
        m_currentWidth = m_widthMax;

        res = m_cameraHandle->GetFeatureByName("HeightMax", pFormatFeature);
        if (VmbErrorSuccess == res) {
            res = pFormatFeature->GetValue(m_heightMax);
        } else {
            m_heightMax = -1;
        }
        m_currentHeight = m_heightMax;

        /// Set image offset to 0
        res = m_cameraHandle->GetFeatureByName("OffsetX", pFormatFeature);
        if (VmbErrorSuccess == res) {
            res = pFormatFeature->SetValue( 0 );
        }
        m_currentXOffset = 0;

        res = m_cameraHandle->GetFeatureByName("OffsetY", pFormatFeature);
        if (VmbErrorSuccess == res) {
            res = pFormatFeature->SetValue( 0 );
        }
        m_currentYOffset = 0;

        /// Set the width
        res = m_cameraHandle->GetFeatureByName("Width", pFormatFeature);
        if (VmbErrorSuccess != res) {
            qWarning() << "failed getting 'Width' feature";
        }

        res = pFormatFeature->SetValue( m_currentWidth );
        if (VmbErrorSuccess != res) {
            qWarning() << "failed setting 'Width' feature to" << m_currentWidth;
        }

        /// Set height
        res = m_cameraHandle->GetFeatureByName( "Height", pFormatFeature );
        if (VmbErrorSuccess != res) {
            qWarning() << "failed getting 'Height' feature";
        }

        pFormatFeature->SetValue( m_currentHeight );
        if (VmbErrorSuccess != res) {
            qWarning() << "failed setting 'Height' feature to"  << m_currentHeight;
        }

        /// Set pixel format. For the sake of simplicity we only support Mono and RGB in this example.
        res = m_cameraHandle->GetFeatureByName( "PixelFormat", pFormatFeature );
        if (VmbErrorSuccess != res) {
            qWarning() << "failed getting 'PixelFormat' feature";
        }

        /// Try to set Mono8
        res = pFormatFeature->SetValue( VmbPixelFormatMono8 );
        if (VmbErrorSuccess != res) {
            qWarning() << "failed setting 'PixelFormat' feature to MONO8";
        }

        /// Read back the currently selected pixel format
        pFormatFeature->GetValue( m_nPixelFormat );
        if (VmbErrorSuccess != res) {
            qWarning() << "failed getting current 'PixelFormat' value";
        }

        /// close camera
        m_cameraHandle->Close();
    }
*/

    /// create a thread for the camera and move the camera to it
    QThread *cameraThread = new QThread();
    qDebug() << qPrintable(
                    QString("[%1] moving camera to its own thread (0x%2)")
                    .arg(this->uuid())
                    .arg((long)cameraThread, 0, 16));
    this->moveToThread(cameraThread);
    /// start thread with the highest priority
    cameraThread->start(QThread::TimeCriticalPriority);

    ///
    qDebug() << "created camera:" << uuid();
}

AVTVimbaCamera::~AVTVimbaCamera()
{
    close();
}

QString AVTVimbaCamera::deviceID() const
{
    return m_deviceID;
}

bool AVTVimbaCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition())
        return false;

    qDebug() << Q_FUNC_INFO;

    /// Open the desired camera
    open();

    VmbErrorType res;

    /// Start capturing images
    if (m_opened) {
        /// Create a frame observer for this camera (This will be wrapped in a shared_ptr so we don't delete it)
        m_frameObserverInternal = new FrameObserver(m_cameraHandle);

        /// Connect frame ready event with event handler
        QObject::connect(m_frameObserverInternal.data(), SIGNAL(FrameReceivedSignal(int)),
                         this, SLOT(onFrameReady(int)));

        /// Start streaming
        res = m_cameraHandle->StartContinuousImageAcquisition(m_vimbaBufferSize, AVT::VmbAPI::IFrameObserverPtr( m_frameObserverInternal.data() ));
        if (VmbErrorSuccess == res)
            return true;
    }

    qWarning() << uuid() << "unable to start acquisition:" << res;
    return false;
}

bool AVTVimbaCamera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition())
        return false;

    qDebug() << Q_FUNC_INFO;

    /// Stop streaming
    m_cameraHandle->StopContinuousImageAcquisition();

    /// clear queue
    if (m_frameObserverInternal)
        m_frameObserverInternal->ClearFrameQueue();

    /// Close camera
    close();

    return true;
}

QList<ZCameraInterface::ZCameraAttribute> AVTVimbaCamera::getAllAttributes()
{
    QList<ZCameraInterface::ZCameraAttribute> attributeList;

    VmbErrorType err;

    bool opened = open();

    AVT::VmbAPI::FeaturePtrVector features;
    err = m_cameraHandle->GetFeatures(features);
    if (VmbErrorSuccess != err) {
        qWarning() << "m_cameraHandle->GetFeatures() failed. Error" << err;
        return attributeList;
    }

    for (unsigned int i=0; i<features.size(); ++i) {
        ZCameraInterface::ZCameraAttribute attribute;

        AVT::VmbAPI::FeaturePtr feature = features[i];

        QString qcategory;
        std::string stdstring;
        if (VmbErrorSuccess == feature->GetCategory(stdstring)) {
            qcategory = QString::fromStdString(stdstring);
            if (qcategory.startsWith('/')) {
                qcategory = qcategory.mid(1);
            }
        }

        if (VmbErrorSuccess == feature->GetName(stdstring))
            attribute.name = QString("Device::%1::%2")
                    .arg(qcategory.replace('/', "::"))
                    .arg(QString::fromStdString(stdstring));

        if (VmbErrorSuccess == feature->GetDescription(stdstring))
            attribute.description = QString::fromStdString(stdstring);

        VmbFeatureFlagsType flags;
        if (VmbErrorSuccess == feature->GetFlags(flags)) {
            if (flags & VmbFeatureFlagsRead)
                attribute.readable = true;
            if (flags & VmbFeatureFlagsWrite)
                attribute.writable = true;
        }

        VmbFeatureDataType dataType;
        if (VmbErrorSuccess == feature->GetDataType(dataType)) {
            switch (dataType) {
            case VmbFeatureDataBool:
                attribute.type = ZCameraInterface::CameraAttributeTypeBool;
                bool boolValue;
                if (VmbErrorSuccess == feature->GetValue(boolValue))
                    attribute.value = boolValue;
                else
                    qWarning() << "feature->GetValue() failed." << attribute.name;
                break;
            case VmbFeatureDataCommand:
                attribute.type = ZCameraInterface::CameraAttributeTypeCommand;
                /// commands needs to be readable to enable them in the settings widget
                attribute.readable = true;
                break;
            case VmbFeatureDataEnum: {
                attribute.type = ZCameraInterface::CameraAttributeTypeEnum;

                VmbInt64_t int64Value = -1;
                VmbInt64_t int64EnumValue = -1;
                if (VmbErrorSuccess != feature->GetValue(int64Value))
                    qWarning() << "feature->GetValue() failed." << attribute.name;
                //else
                    //attribute.enumValue = int64Value;

                AVT::VmbAPI::EnumEntryVector entries;
                if (VmbErrorSuccess == feature->GetEntries(entries)) {
                    for (unsigned int j=0; j<entries.size(); ++j) {
                        AVT::VmbAPI::EnumEntry entry = entries[j];
                        if (VmbErrorSuccess == entry.GetName(stdstring))
                            attribute.enumNames << QString::fromStdString(stdstring);
                        else
                            attribute.enumNames << QString("[error getting entry name]");
                        if (VmbErrorSuccess == entry.GetValue(int64EnumValue)
                                && int64EnumValue == int64Value)
                            attribute.enumValue = j;
                    }
                }

                break;
            }
            case VmbFeatureDataFloat:
                attribute.type = ZCameraInterface::CameraAttributeTypeFloat;
                double doubleValue;
                if (VmbErrorSuccess == feature->GetValue(doubleValue))
                    attribute.value = doubleValue;
                else
                    qWarning() << "feature->GetValue() failed." << attribute.name;
                break;
            case VmbFeatureDataInt: {
                attribute.type = ZCameraInterface::CameraAttributeTypeInt;
                VmbInt64_t int64Value;
                if (VmbErrorSuccess == feature->GetValue(int64Value))
                    attribute.value = int64Value;
                else
                    qWarning() << "feature->GetValue() failed." << attribute.name;
                break;
            }
            case VmbFeatureDataString:
                attribute.type = ZCameraInterface::CameraAttributeTypeString;
                if (VmbErrorSuccess == feature->GetValue(stdstring))
                    attribute.value = QString::fromStdString(stdstring);
                else
                    qWarning() << "feature->GetValue() failed." << attribute.name;
                break;
            case VmbFeatureDataNone:
            case VmbFeatureDataRaw:
            case VmbFeatureDataUnknown:
            default:
                qWarning() << "unknown VmbFeatureDataType:" << dataType;
            }
        }

        /*
        VmbFeatureVisibilityType visibility;
        if (VmbErrorSuccess == feature->GetVisibility(visibility)) {

        }
        */

        VmbInt64_t minimum, maximum;
        if (VmbErrorSuccess == feature->GetRange(minimum, maximum)) {
            attribute.minimumValue = minimum;
            attribute.maximumValue = maximum;
        }

        attributeList << attribute;
    }

    if (opened)
        close();

    return attributeList;
}

QVariant AVTVimbaCamera::getAttribute(const QString &name) const
{
    //! TODO
    return QVariant("INVALID");
}

bool AVTVimbaCamera::setBufferSize(int bufferSize)
{
    if (ZCameraBase::setBufferSize(bufferSize)) {
        m_vimbaBufferSize = bufferSize;
        return true;
    } else
        return false;
}

bool AVTVimbaCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    //qDebug() << "trying to set attribute" << name << "to" << value;

    bool changed = false;

    bool opened = open();

    int index = name.lastIndexOf("::");
    if (index >= 0) {
        index += 2;
    }

    AVT::VmbAPI::FeaturePtr feature;
    VmbErrorType err = m_cameraHandle->GetFeatureByName(qPrintable(name.mid(index)), feature);
    if (VmbErrorSuccess == err) {
        VmbFeatureDataType dataType;
        if (VmbErrorSuccess == feature->GetDataType(dataType)) {
            switch (dataType) {
            case VmbFeatureDataBool: {
                bool boolValue = value.toBool();
                if (VmbErrorSuccess == feature->SetValue(boolValue))
                    changed = true;
                break;
            }
            case VmbFeatureDataCommand: {
                if (VmbErrorSuccess == feature->RunCommand()) {
                    changed = true;
                    bool bIsCommandDone = false;
                    do {
                        if (VmbErrorSuccess != feature->IsCommandDone( bIsCommandDone )) {
                            break;
                        }
                    } while (false == bIsCommandDone);
                }
                break;
            }
            case VmbFeatureDataEnum: {
                VmbInt64_t int64Value = -1;
                std::string stdstring;
                AVT::VmbAPI::EnumEntryVector entries;
                if (VmbErrorSuccess == feature->GetEntries(entries)) {
                    for (unsigned int j=0; j<entries.size(); ++j) {
                        AVT::VmbAPI::EnumEntry entry = entries[j];
                        if (VmbErrorSuccess == entry.GetName(stdstring)) {
                            if (QString::fromStdString(stdstring) == value.toString()) {
                                entry.GetValue(int64Value);
                                if (VmbErrorSuccess == feature->SetValue(int64Value))
                                    changed = true;
                                break;
                            }
                        } else {
                            qWarning() << "error getting entry name" << j << "for" << name;
                        }
                    }
                }
                break;
            }
            case VmbFeatureDataFloat: {
                double doubleValue = value.toDouble();
                if (VmbErrorSuccess == feature->SetValue(doubleValue))
                    changed = true;
                break;
            }
            case VmbFeatureDataInt: {
                VmbInt64_t int64Value = value.toLongLong();
                if (VmbErrorSuccess == feature->SetValue(int64Value))
                    changed = true;
                break;
            }
            case VmbFeatureDataString: {
                if (VmbErrorSuccess == feature->SetValue(qPrintable(value.toString())))
                    changed = true;
                break;
            }
            case VmbFeatureDataNone:
            case VmbFeatureDataRaw:
            case VmbFeatureDataUnknown:
            default:
                qWarning() << "unknown VmbFeatureDataType:" << dataType;
            }
        }
    } else {
        qWarning() << "unable to get feature" << name << ". Error:" << err;
    }

    if (opened)
        close();

    if (changed) {
        if (notify)
            emit attributeChanged(name, value);
    } else {
        qWarning() << "unable to set attribute" << name << "to" << value;
    }

    return changed;
}

void AVTVimbaCamera::onFrameReady(int status)
{
    /// Pick up frame
    AVT::VmbAPI::FramePtr pFrame = m_frameObserverInternal->GetFrame();

    /// See if it is not corrupt
    if (VmbFrameStatusComplete == status ) {
        VmbPixelFormatType pixelFormat;
        if (VmbErrorSuccess != pFrame->GetPixelFormat(pixelFormat))
            qWarning() << "failed getting pixel format!";

        int bytesPerPixel = -1;
        switch (pixelFormat) {
        case VmbPixelFormatMono8:
            bytesPerPixel = 1;
            break;
        case VmbPixelFormatMono10:
        case VmbPixelFormatMono12:
        case VmbPixelFormatMono14:
        case VmbPixelFormatMono16:
            bytesPerPixel = 2;
            break;
        default:
            qWarning() << "unknown pixel format. skipping image";
        }

        if (bytesPerPixel > 0) {
            /// get image data
            VmbUchar_t *pBuffer;
            VmbErrorType err = pFrame->GetImage( pBuffer );
            if (err != VmbErrorSuccess) {
                qWarning() << uuid() << "failure in retrieving image buffer" << err;
            } else {
                VmbUint32_t value;
                if (VmbErrorSuccess == pFrame->GetHeight(value))
                    m_currentHeight = value;

                if (VmbErrorSuccess == pFrame->GetWidth(value))
                    m_currentWidth = value;

                if (VmbErrorSuccess == pFrame->GetOffsetX(value))
                    m_currentXOffset = value;

                if (VmbErrorSuccess == pFrame->GetOffsetY(value))
                    m_currentYOffset = value;


                /// get next image from buffer
                /*ImageGrayscale::Ptr &currentImage = getNextBufferImage();

                /// this was used when we copied driver image buffer to own image buffer
                /// this was useless, we could increase the buffer size and we'll
                /// have the same results and reduce the data copying overhead

                /// if null or different size
                if (currentImage.isNull() ||
                        currentImage->width() != m_currentWidth ||
                        currentImage->height() != m_currentHeight ||
                        currentImage->bytesPerPixel() != bytesPerPixel) {
                    currentImage.swap( ImageGrayscale::Ptr(new ImageGrayscale(m_currentWidth, m_currentHeight,
                                                                              m_currentXOffset, m_currentYOffset,
                                                                              bytesPerPixel)) );
                }

                /// copy data
                currentImage->setBuffer(pBuffer);


                /// This is without copying data, use same buffer
                ImageGrayscale::Ptr newImage(new ImageGrayscale(m_currentWidth, m_currentHeight,
                                                                m_currentXOffset, m_currentYOffset,
                                                                bytesPerPixel,
                                                                pBuffer));

                currentImage.swap( newImage );
*/

                /*
                /// This is without copying data, use same buffer
                ImageGrayscale::Ptr currentImage = getNextBufferImage(m_currentWidth, m_currentHeight,
                                                                      m_currentXOffset, m_currentYOffset,
                                                                      bytesPerPixel,
                                                                      pBuffer);
                */

                ZImageGrayscale::Ptr currentImage = getNextBufferImage(m_currentWidth, m_currentHeight,
                                                                      m_currentXOffset, m_currentYOffset,
                                                                      bytesPerPixel);

                /// copy data
                currentImage->setBuffer(pBuffer);

                /// set image number
                VmbUint64_t frameID;
                pFrame->GetFrameID(frameID);
                currentImage->setNumber(frameID);

                /// notify
                emit newImageReceived(currentImage);
            }
        }
    } else {
        /// If we receive an incomplete image we do nothing but logging
        qWarning() << uuid() << "failure in receiving image" << (VmbErrorType)status;
    }

    /// And queue it to continue streaming
    m_cameraHandle->QueueFrame( pFrame );
}

bool AVTVimbaCamera::open()
{
    if (!m_opened) {
        VmbErrorType err = m_cameraHandle->Open(VmbAccessModeFull);
        if (VmbErrorSuccess == err) {
            m_opened = true;
            return true;
        } else {
            qWarning() << "m_cameraHandle->Open(VmbAccessModeFull) failed. Error" << err;
        }
    }

    return false;
}

bool AVTVimbaCamera::close()
{
    if (m_opened) {
        VmbErrorType err = m_cameraHandle->Close();
        if (VmbErrorSuccess == err) {
            m_opened = false;
            return true;
        } else {
            qWarning() << "m_cameraHandle->Close() failed. Error" << err;
        }
    }

    return false;
}

} // namespace Z3D
