#include "zpleoraebuscamera_p.h"

#include <QDebug>
#include <QTime>
#include <QTimer>
#include <QThread>

#include "stdint.h"

#include <PvInterface.h>




#include <PvBuffer.h>
//#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvSystem.h>



// defines to be used as "Top Level" categories
#define PARAM_DEVICE "Device"
#define PARAM_STREAM "Stream"
#define PARAM_COMMUNICATION "Communication"


//!
#define DEBUG_CHUNK_MODE

#ifdef DEBUG_CHUNK_MODE
#  include <QFile>
#  include "at_c4_2040_chunkmodedefs.h"
#endif


PvSystem *Z3D::ZPleoraeBUSCameraPrivate::lSystem = NULL;



PvStream *OpenStream( const PvDeviceInfo *aDeviceInfo )
{
    PvStream *lStream;
    PvResult lResult;

    /// Open stream to the GigE Vision or USB3 Vision device
    qDebug() << "Opening stream to device.";
    lStream = PvStream::CreateAndOpen( aDeviceInfo->GetConnectionID(), &lResult );
    if ( lStream == NULL ) {
        qDebug() << "Unable to stream from " << aDeviceInfo->GetDisplayID().GetAscii();
    }

    return lStream;
}

void ConfigureStream( PvDevice *aDevice, PvStream *aStream )
{
    /// If this is a GigE Vision device, configure GigE Vision specific streaming parameters
    PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV *>( aDevice );
    if ( lDeviceGEV != NULL ) {
        PvStreamGEV *lStreamGEV = static_cast<PvStreamGEV *>( aStream );

        /// Negotiate packet size
        lDeviceGEV->NegotiatePacketSize();

        /// Configure device streaming destination
        lDeviceGEV->SetStreamDestination( lStreamGEV->GetLocalIPAddress(), lStreamGEV->GetLocalPort() );
    }
}

void CreateStreamBuffers( PvDevice *aDevice, PvStream *aStream, BufferList *aBufferList, unsigned int BUFFER_COUNT)
{
    /// Reading payload size from device
    uint32_t lSize = aDevice->GetPayloadSize();

    /// Use BUFFER_COUNT or the maximum number of buffers, whichever is smaller
    uint32_t lBufferCount = ( aStream->GetQueuedBufferMaximum() < BUFFER_COUNT ) ?
        aStream->GetQueuedBufferMaximum() :
        BUFFER_COUNT;

    /// Allocate buffers
    for ( uint32_t i = 0; i < lBufferCount; i++ ) {
        /// Create new buffer object
        PvBuffer *lBuffer = new PvBuffer;

        /// Have the new buffer object allocate payload memory
        lBuffer->Alloc( static_cast<uint32_t>( lSize ) );

        /// Add to external list - used to eventually release the buffers
        aBufferList->push_back( lBuffer );
    }

//    uint32_t lMax = aStream->GetQueuedBufferMaximum();

    /// Queue all buffers in the stream
    BufferList::iterator lIt = aBufferList->begin();
    while ( lIt != aBufferList->end() ) {
        aStream->QueueBuffer( *lIt );
        lIt++;
    }
}

void FreeStreamBuffers( BufferList *aBufferList )
{
    /// Go through the buffer list
    BufferList::iterator lIt = aBufferList->begin();
    while ( lIt != aBufferList->end() ) {
        delete *lIt;
        lIt++;
    }

    /// Clear the buffer list
    aBufferList->clear();
}






namespace Z3D
{

ZPleoraeBUSCameraPrivate *ZPleoraeBUSCameraPrivate::getCameraByMAC(QString name)
{
    if (!lSystem)
        lSystem = new PvSystem;

    PvResult lResult;
    //const PvDeviceInfo* lLastDeviceInfo = NULL;


    const PvDeviceInfo *lDeviceInfo;
    lResult = lSystem->FindDevice(PvString(qPrintable(name)), &lDeviceInfo);
    if (lResult.IsOK())
        return new ZPleoraeBUSCameraPrivate(lDeviceInfo);
    else
        return 0;



    /*
    /// Find all devices on the network.
    static PvSystem lSystem;
    lResult = lSystem.Find();
    if ( !lResult.IsOK() ) {
        qWarning() << "PvSystem::Find Error: " << lResult.GetCodeString().GetAscii();
        return 0;
    }

    /// Go through all interfaces
    uint32_t lInterfaceCount = lSystem.GetInterfaceCount();
    for ( uint32_t x = 0; x < lInterfaceCount; x++ ) {
        //cout << "Interface " << x << endl;

        /// Get pointer to the interface.
        const PvInterface* lInterface = lSystem.GetInterface( x );

        /// Is it a PvNetworkAdapter?
        const PvNetworkAdapter* lNIC = dynamic_cast<const PvNetworkAdapter*>( lInterface );
        if ( lNIC != NULL ) {
//            cout << "  MAC Address: " << lNIC->GetMACAddress().GetAscii() << endl;
//            cout << "  IP Address: " << lNIC->GetIPAddress().GetAscii() << endl;
//            cout << "  Subnet Mask: " << lNIC->GetSubnetMask().GetAscii() << endl << endl;
        }

        /// Is it a PvUSBHostController?
        const PvUSBHostController* lUSB = dynamic_cast<const PvUSBHostController*>( lInterface );
        if ( lUSB != NULL ) {
//            cout << "  Name: " << lUSB->GetName().GetAscii() << endl << endl;
        }

        /// Go through all the devices attached to the interface
        uint32_t lDeviceCount = lInterface->GetDeviceCount();
        for ( uint32_t y = 0; y < lDeviceCount ; y++ ) {
            const PvDeviceInfo *lDeviceInfo = lInterface->GetDeviceInfo( y );

            //cout << "  Device " << y << endl;
            //cout << "    Display ID: " << lDeviceInfo->GetDisplayID().GetAscii() << endl;

            const PvDeviceInfoGEV* lDeviceInfoGEV = dynamic_cast<const PvDeviceInfoGEV*>( lDeviceInfo );
            const PvDeviceInfoU3V *lDeviceInfoU3V = dynamic_cast<const PvDeviceInfoU3V *>( lDeviceInfo );
            const PvDeviceInfoUSB *lDeviceInfoUSB = dynamic_cast<const PvDeviceInfoUSB *>( lDeviceInfo );
            const PvDeviceInfoPleoraProtocol* lDeviceInfoPleora = dynamic_cast<const PvDeviceInfoPleoraProtocol*>( lDeviceInfo );

            if ( lDeviceInfoGEV != NULL ) { /// Is it a GigE Vision device?

                QString deviceMAC = QString::fromStdString( lDeviceInfoGEV->GetMACAddress().GetAscii() );
                if (deviceMAC.toUpper() == name.toUpper()) {
                    return new ZPleoraeBUSCameraPrivate(lDeviceInfoGEV);
                }

                qDebug() << "    MAC Address: " << lDeviceInfoGEV->GetMACAddress().GetAscii();
                qDebug() << "    IP Address: " << lDeviceInfoGEV->GetIPAddress().GetAscii();
                qDebug() << "    Serial number: " << lDeviceInfoGEV->GetSerialNumber().GetAscii();
//                lLastDeviceInfo = lDeviceInfo;
            } else if ( lDeviceInfoU3V != NULL ) { /// Is it a USB3 Vision device?
//                cout << "    GUID: " << lDeviceInfoU3V->GetDeviceGUID().GetAscii() << endl;
//                cout << "    S/N: " << lDeviceInfoU3V->GetSerialNumber().GetAscii() << endl;
//                cout << "    Speed: " << lUSB->GetSpeed() << endl << endl;
//                lLastDeviceInfo = lDeviceInfo;
            } else if ( lDeviceInfoUSB != NULL ) { /// Is it an unidentified USB device?
//                cout << endl;
            } else if ( lDeviceInfoPleora != NULL ) { /// Is it a Pleora Protocol device?
//                cout << "    MAC Address: " << lDeviceInfoPleora->GetMACAddress().GetAscii() << endl;
//                cout << "    IP Address: " << lDeviceInfoPleora->GetIPAddress().GetAscii() << endl;
//                cout << "    Serial number: " << lDeviceInfoPleora->GetSerialNumber().GetAscii() << endl << endl;
            }
        }
    }

    return 0;*/
}

ZPleoraeBUSCameraPrivate::ZPleoraeBUSCameraPrivate(const PvDeviceInfo *deviceInfo, QObject *parent) :
    QObject(parent),
    lDeviceInfo(deviceInfo),
    lDevice(0),
    lStream(0),
    m_internalBufferSize(20),
    m_acquisitionCounter(0),
    m_stopThreadRequested(true)
{
    /// obtain properties to generate unique camera id
    m_uuid += QString("ZPleoraeBUS-%1-%2")
            .arg(lDeviceInfo->GetModelName().GetAscii())
            .arg(lDeviceInfo->GetSerialNumber().GetAscii());

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

    PvResult lResult;
    lDevice = PvDevice::CreateAndConnect( lDeviceInfo, &lResult );
    if ( !lResult.IsOK() ) {
        qWarning() << "Unable to connect to " << lDeviceInfo->GetDisplayID().GetAscii() << endl;
    } else {
        qDebug() << "Successfully connected to " << lDeviceInfo->GetDisplayID().GetAscii() << endl;

        /// Get device parameters need to control streaming
        lDeviceParams = lDevice->GetParameters();
        /*
        lTLLocked = dynamic_cast<PvGenInteger *>( lDeviceParams->Get( "TLParamsLocked" ) );
        lPayloadSize = dynamic_cast<PvGenInteger *>( lDeviceParams->Get( "PayloadSize" ) );
        lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
        lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );*/
    }
}

ZPleoraeBUSCameraPrivate::~ZPleoraeBUSCameraPrivate()
{
    qDebug() << "destroying camera:" << m_uuid;

    if (lDevice) {
        /// stop acquisition
        stopAcquisition();

        /// finally disconnect the device. Optional, still nice to have
        qDebug() << "Disconnecting device";
        lDevice->Disconnect();

        PvDevice::Free( lDevice );
    }
}

bool ZPleoraeBUSCameraPrivate::startAcquisition()
{
    qDebug() << Q_FUNC_INFO;

    m_currentAcquisitionCounter = m_acquisitionCounter;
    m_lastReturnedBufferNumber = 0;

    ////////////////////////////////////////////////////////////////////////////
    lStream = OpenStream( lDeviceInfo );
    if ( NULL == lStream ) {
        /// something failed, stop acquisition
        q_ptr->stopAcquisition();
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    ConfigureStream( lDevice, lStream );
    CreateStreamBuffers( lDevice, lStream, &lBufferList, m_internalBufferSize );

    /// start running grab loop in the objects thread
    m_stopThreadRequested = false;
    QTimer::singleShot(0, this, SLOT(grabLoop()));

    return true;
}

bool ZPleoraeBUSCameraPrivate::stopAcquisition()
{
    m_stopThreadRequested = true;

    qDebug() << Q_FUNC_INFO;
    qDebug() << "last returned buffer number:" << m_lastReturnedBufferNumber;

    m_acquisitionCounter++;
    if (m_acquisitionCounter > INT_MAX/2)
        m_acquisitionCounter = 0;

    return true;
}

QList<Z3D::ZCameraInterface::ZCameraAttribute> ZPleoraeBUSCameraPrivate::getAllAttributes(PvGenParameterArray *paramsArray, QString topCategory)
{
    QList<Z3D::ZCameraInterface::ZCameraAttribute> setts;

    for (uint32_t i=0; i<paramsArray->GetCount(); ++i) {
        PvGenParameter *lGenParameter = paramsArray->Get(i);

        PvString pvString;
        QString category;
        if (!topCategory.isEmpty()) {
            category = topCategory + "::";
        }
        if (lGenParameter->GetCategory(pvString).IsOK()) {
            QString pleoraCategory = QString::fromStdString(pvString.GetAscii());
            /// remove "Root" category
            if (pleoraCategory.startsWith("Root\\"))
                pleoraCategory = pleoraCategory.mid(5);
            /// pleora uses "\" to separate categories, we use "::"
            category += pleoraCategory.replace('\\', "::");
        }

        QString name = QString::fromStdString(lGenParameter->GetName().GetAscii());

        if (!lGenParameter->IsAvailable()) {
            //qDebug() << "Parameter" << category + "::" + name << "not available.";
            continue;
        }

        Z3D::ZCameraInterface::ZCameraAttribute attr;
        attr.name = category + "::" + name;

        if (lGenParameter->GetDescription(pvString).IsOK())
            attr.description = QString::fromStdString(pvString.GetAscii());

        attr.readable = lGenParameter->IsReadable();
        attr.writable = lGenParameter->IsWritable();

        PvGenType pvGenType;
        if (lGenParameter->GetType(pvGenType).IsOK()) {
            switch (pvGenType) {
            case PvGenTypeBoolean: {
                attr.type = Z3D::ZCameraInterface::CameraAttributeTypeBool;
                bool lValue;
                static_cast<PvGenBoolean *>( lGenParameter )->GetValue( lValue );
                attr.value = lValue;
                break;
            }
            case PvGenTypeCommand: {
                attr.type = Z3D::ZCameraInterface::CameraAttributeTypeCommand;
                /// commands aren't readable, but we must set readable to
                /// enable them when they are writable (executable)
                attr.readable = true;
                break;
            }
            case PvGenTypeEnum: {
                attr.type = Z3D::ZCameraInterface::CameraAttributeTypeEnum;
                PvGenEnum *pvGen = static_cast<PvGenEnum *>(lGenParameter);
                int64_t enumValue = -1,
                        value = -2;
                if (!pvGen->GetValue(value).IsOK())
                    qWarning() << "failed getting enum value for" << attr.name;
                int64_t count;
                if (pvGen->GetEntriesCount(count).IsOK()) {
                    const PvGenEnumEntry *entry;
                    for (int64_t c=0; c<count; ++c) {
                        if (pvGen->GetEntryByIndex(c, &entry).IsOK()) {
                            if (entry->GetDisplayName(pvString).IsOK() || entry->GetName(pvString).IsOK()) {
                                attr.enumNames << QString::fromStdString(pvString.GetAscii());
                            } else {
                                attr.enumNames << "Unable to get enum name";
                            }
                            if (entry->GetValue(enumValue).IsOK()) {
                                if (enumValue == value) {
                                    attr.enumValue = c;
                                }
                            } else {
                                qWarning() << "failed getting enum value for" << attr.enumNames.last();
                            }
                        } else {
                            attr.enumNames << "Unable to get enum entry";
                        }
                    }
                }
                break;
            }
            case PvGenTypeFloat: {
                attr.type = Z3D::ZCameraInterface::CameraAttributeTypeFloat;
                double lValue;
                static_cast<PvGenFloat *>( lGenParameter )->GetValue( lValue );
                attr.value = lValue;
                break;
            }
            case PvGenTypeInteger: {
                attr.type = Z3D::ZCameraInterface::CameraAttributeTypeInt;
                int64_t lValue;
                static_cast<PvGenInteger *>( lGenParameter )->GetValue( lValue );
                attr.value = lValue;
                break;
            }
            case PvGenTypeString: {
                attr.type = Z3D::ZCameraInterface::CameraAttributeTypeString;
                PvString lValue;
                static_cast<PvGenString *>( lGenParameter )->GetValue( lValue );
                attr.value = QString::fromStdString(lValue.GetAscii());
                break;
            }
            case PvGenTypeRegister:
                //qWarning() << "PvGenTypeRegister not supported yet, param:" << attr.name;
                break;
            case PvGenTypeUndefined:
                //qWarning() << "PvGenTypeUndefined not supported yet, param:" << attr.name;
                break;
            default:
                //qWarning() << "Unknown PvGenType, param:" << attr.name;
                break;
            }
        }

        setts << attr;
    }

    return setts;
}

QList<Z3D::ZCameraInterface::ZCameraAttribute> ZPleoraeBUSCameraPrivate::getAllAttributes()
{
    QList<Z3D::ZCameraInterface::ZCameraAttribute> setts;

    /// device parameters should be always available
    setts << getAllAttributes(lDeviceParams, PARAM_DEVICE);

    /// image stream parameters, when stream is available
    if (lStream)
        setts << getAllAttributes(lStream->GetParameters(), PARAM_STREAM);

    /// communication link parameters
    setts << getAllAttributes( lDevice->GetCommunicationParameters(), PARAM_COMMUNICATION );

    return setts;
}

QVariant ZPleoraeBUSCameraPrivate::getAttribute(const QString &/*name*/) const
{
    QVariant value;

    //! FIXME TODO

    return value;
}

bool ZPleoraeBUSCameraPrivate::setBufferSize(int bufferSize)
{
    m_internalBufferSize = bufferSize;
    return true;
}

bool ZPleoraeBUSCameraPrivate::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    // qDebug() << "setting attribute" << name << ":" << value;

    bool changed = false;

    int index = name.lastIndexOf("::");
    if (index >= 0) {
        index += 2;
    }

    PvGenParameter *param = NULL;

    if (name.startsWith(PARAM_DEVICE)) {
        param = lDevice->GetParameters()->Get(PvString(qPrintable(name.mid(index))));
    } else if (name.startsWith(PARAM_COMMUNICATION)) {
        param = lDevice->GetCommunicationParameters()->Get(PvString(qPrintable(name.mid(index))));
    } else if (name.startsWith(PARAM_STREAM)) {
        if (lStream) {
            param = lStream->GetParameters()->Get(PvString(qPrintable(name.mid(index))));
        }
    } else {
        qWarning() << "unknown param type:" << name;
    }

    if (param) {
        PvGenType pvGenType;
        if (param->GetType(pvGenType).IsOK()) {
            switch (pvGenType) {
            case PvGenTypeBoolean: {
//                qDebug() << "PvGenTypeBoolean, param:" << name << ", value:" << value.toBool();
                PvGenBoolean *pvGen = dynamic_cast<PvGenBoolean *>(param);
                if (pvGen->SetValue(value.toBool()).IsOK())
                    changed = true;
                break;
            }
            case PvGenTypeCommand: {
//                qDebug() << "PvGenTypeCommand, param:" << name << ", value:" << value.toString();
                PvGenCommand *pvGen = dynamic_cast<PvGenCommand *>(param);
                if (pvGen->Execute().IsOK())
                    changed = true;
                break;
            }
            case PvGenTypeEnum: {
//                qDebug() << "PvGenTypeEnum, param:" << name << ", value:" << value.toString();
                PvGenEnum *pvGen = dynamic_cast<PvGenEnum *>(param);
                int64_t enumValue = -1;
                int64_t count;
                QString valueStr = value.toString();
                if (pvGen->GetEntriesCount(count).IsOK()) {
                    const PvGenEnumEntry *entry;
                    bool found = false;
                    PvString pvString;
                    for (int64_t c=0; c<count; ++c) {
                        if (pvGen->GetEntryByIndex(c, &entry).IsOK()) {
                            found = false;
                            if (entry->GetDisplayName(pvString).IsOK()) {
                                if (valueStr == QString::fromStdString(pvString.GetAscii()))
                                    found = true;
                            } else {
                                qWarning() << "Unable to get enum display name:" << name;
                            }
                            if (!found) {
                                if (entry->GetName(pvString).IsOK()) {
                                    if (valueStr == QString::fromStdString(pvString.GetAscii()))
                                        found = true;
                                } else {
                                    qWarning() << "Unable to get enum name:" << name;
                                }
                            }
                            if (found) {
                                if (entry->GetValue(enumValue).IsOK()) {
                                    if (pvGen->SetValue(enumValue).IsOK())
                                        changed = true;
                                    break;
                                } else {
                                    qWarning() << "failed getting enum value for" << name;
                                }
                            }
                        } else {
                            qWarning() << "Unable to get enum entry:" << name;
                        }
                    }
                }

                /// last try, set value as is
                if (pvGen->SetValue(PvString(qPrintable(value.toString()))).IsOK())
                    changed = true;

                break;
            }
            case PvGenTypeFloat: {
                bool ok = false;
                double doubleValue = value.toDouble(&ok);
//                qDebug() << "PvGenTypeFloat, param:" << name << ", value:" << doubleValue;
                if (ok) {
                    PvGenFloat *pvGen = dynamic_cast<PvGenFloat *>(param);
                    if (pvGen->SetValue(doubleValue).IsOK())
                        changed = true;
                } else {
                    qWarning() << "failed converting value to double:" << value;
                }
                break;
            }
            case PvGenTypeInteger: {
//                qDebug() << "PvGenTypeInteger, param:" << name << ", value:" << value.toInt();
                PvGenInteger *pvGen = dynamic_cast<PvGenInteger *>(param);
                if (pvGen->SetValue(value.toInt()).IsOK())
                    changed = true;
                break;
            }
            case PvGenTypeString: {
//                qDebug() << "PvGenTypeString, param:" << name << ", value:" << value.toString();
                PvGenString *pvGen = dynamic_cast<PvGenString *>(param);
                if (pvGen->SetValue(PvString(qPrintable(value.toString()))).IsOK())
                    changed = true;
                break;
            }
            case PvGenTypeRegister:
                qWarning() << "PvGenTypeRegister not supported yet, param:" << name;
                break;
            case PvGenTypeUndefined:
                qWarning() << "PvGenTypeUndefined not supported yet, param:" << name;
                break;
            default:
                qWarning() << "Unknown PvGenType, param:" << name;
                break;
            }
        }
    } else {
        qWarning() << "unable to get param:" << name;
    }

    if (notify && changed) {
        /// update properties widget
        emit attributeChanged(name, value);
    }

    if (!changed)
        qWarning() << "failed setting attribute" << name << "to" << value;

    /// return true, status == success
    return changed;
}

void ZPleoraeBUSCameraPrivate::grabLoop()
{
    //QTime processingTime;
    //processingTime.start();

    /// Get device parameters need to control streaming
    PvGenParameterArray *lDeviceParams = lDevice->GetParameters();

    /// Map the GenICam AcquisitionStart and AcquisitionStop commands
    PvGenCommand *lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
    PvGenCommand *lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );

    /// Get stream parameters
//    PvGenParameterArray *lStreamParams = lStream->GetParameters();

    /// Map a few GenICam stream stats counters
//    PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "AcquisitionRate" ) );
//    PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "Bandwidth" ) );

    /// Enable streaming and send the AcquisitionStart command
    qDebug() << "Enabling streaming and sending AcquisitionStart command." << endl;
    lDevice->StreamEnable();
    lStart->Execute();

//    double lFrameRateVal = 0.0;
//    double lBandwidthVal = 0.0;

    /// start grabbing images until either the user tells us to stop or we encounter an error
    while (!m_stopThreadRequested) {
        PvBuffer *lBuffer = NULL;
        PvResult lOperationResult;

        /// Retrieve next buffer
        PvResult lResult = lStream->RetrieveBuffer( &lBuffer, &lOperationResult, 1000 );
        if ( lResult.IsOK() ) {
            if ( lOperationResult.IsOK() ) {
                /// We now have a valid buffer. This is where you would typically process the buffer.
//                lFrameRate->GetValue( lFrameRateVal );
//                lBandwidth->GetValue( lBandwidthVal );

#ifdef DEBUG_CHUNK_MODE
                /// CHUNK TEST
/*
                qDebug() << "lBuffer->GetBlockID()" << lBuffer->GetBlockID()
                         << "lBuffer->GetID()" << lBuffer->GetID()
                         << "lBuffer->GetSize()" << lBuffer->GetSize()
                         << "lBuffer->GetAcquiredSize()" << lBuffer->GetAcquiredSize()
                         << "lBuffer->GetPayloadSize()" << lBuffer->GetPayloadSize()
                         << "lBuffer->GetRequiredSize()" << lBuffer->GetRequiredSize()
                         << "lBuffer->HasChunks()" << lBuffer->HasChunks();

                QFile file1(QString("%1_DataPointer.bin").arg(lBuffer->GetBlockID()));
                if (file1.open(QFile::WriteOnly)) {
                    file1.write((const char*)lBuffer->GetDataPointer(), lBuffer->GetPayloadSize());
                }
//                QFile file2(QString("%1_RawData.bin").arg(lBuffer->GetBlockID()));
//                if (file2.open(QFile::WriteOnly)) {
//                    file2.write((const char*)lBuffer->GetRawData(), lBuffer->GetPayloadSize());
//                }
                QFile file3(QString("%1_ImageDataPointer.bin").arg(lBuffer->GetBlockID()));
                if (file3.open(QFile::WriteOnly)) {
                    PvImage *lImage = lBuffer->GetImage();
                    file3.write((const char*)lImage->GetDataPointer(), lImage->GetImageSize());
                }

                ///
                int chunkCount = lBuffer->GetChunkCount();
                if (lBuffer->HasChunks())
                    qDebug() << "buffer contains chunk data:" << chunkCount;
                else
                    qDebug() << "lBuffer->GetPayloadSize():" << lBuffer->GetPayloadSize();
*/
                /// CHUNK TEST
#endif // DEBUG_CHUNK_MODE

                PvPayloadType lType = lBuffer->GetPayloadType();
                switch (lType) {
                case PvPayloadTypeImage: {
//                    qDebug() << "Buffer contains PvPayloadTypeImage with size" << lBuffer->GetSize();
                    int currentBufferNumber = lBuffer->GetBlockID();

                    //GetID == 0 always?
                    //qDebug() << "GetBlockID():" << currentBufferNumber << "vs GetID():" << lBuffer->GetID();

                    if (currentBufferNumber != m_lastReturnedBufferNumber + 1) {
                        qWarning() << "skipped some frames, previous:" << m_lastReturnedBufferNumber
                                   << "current:" << currentBufferNumber;
                    }

                    m_lastReturnedBufferNumber = currentBufferNumber;

                    /// Get image specific buffer interface
                    PvImage *lImage = lBuffer->GetImage();

                    int bytesPerPixel = -1;
                    switch (lImage->GetBitsPerPixel()) {
                    case 8:
                        bytesPerPixel = 1;
                        break;
                    case 16:
                        bytesPerPixel = 2;
                        break;
                    default:
                        qWarning() << "BitsPerPixel unsupported:" << lImage->GetBitsPerPixel();
                    }

                    uint32_t imgWidth = lImage->GetWidth();
                    uint32_t imgHeight = lImage->GetHeight();
                    if (bytesPerPixel > 0 && imgWidth > 0 && imgHeight > 0) {
                        /// This is copying data
                        ZImageGrayscale::Ptr currentImage = q_ptr->getNextBufferImage(
                                    imgWidth,
                                    imgHeight,
                                    lImage->GetOffsetX(),
                                    lImage->GetOffsetY(),
                                    bytesPerPixel);

                        currentImage->setBuffer(lImage->GetDataPointer());

                        /// This is without copying data, use same buffer
                        /*ImageGrayscale::Ptr currentImage = q_ptr->getNextBufferImage(
                                    lImage->GetWidth(),
                                    lImage->GetHeight(),
                                    lImage->GetOffsetX(),
                                    lImage->GetOffsetY(),
                                    bytesPerPixel,
                                    lImage->GetDataPointer());*/

                        /// set image number
                        currentImage->setNumber(currentBufferNumber);

                        /// notify
                        emit newImageReceived(currentImage);
                    }
#ifdef DEBUG_CHUNK_MODE
                    /// CHUNK TEST
                    else if (lBuffer->GetPayloadSize() > 0) {
                        uint32_t dataSize = lBuffer->GetPayloadSize();
                        uint8_t *data = lBuffer->GetDataPointer();

                        AT::C4_2040::processChunkData(data, dataSize);
                    }
                    /// CHUNK TEST
#endif // DEBUG_CHUNK_MODE

                    break;
                }
                case PvPayloadTypeChunkData: {
                    qDebug() << "Buffer contains PvPayloadTypeChunkData with size" << lBuffer->GetSize();
                    break;
                }
                default:
                    qWarning() << "Buffer of unknown type. PvPayloadType:" << lType;
                }
                //qDebug() << "  " << lFrameRateVal << " FPS  " << ( lBandwidthVal / 1000000.0 ) << " Mb/s";
            } else {
                /// Non OK operational result
                QString message = QString("lStream->RetrieveBuffer not completed successfuly. Error: %1").arg(lOperationResult.GetCodeString().GetAscii());
                emit error(message);
                qDebug() << "lStream->RetrieveBuffer not completed successfuly. Error:" << lOperationResult.GetCodeString().GetAscii();
            }

            /// Re-queue the buffer in the stream object
            lStream->QueueBuffer( lBuffer );
        } else {
            /// Retrieve buffer failure
            QString message = QString("lStream->RetrieveBuffer failed. Error: %1").arg(lResult.GetCodeString().GetAscii());
            emit error(message);
            qWarning() << "lStream->RetrieveBuffer failed. Error:" << lResult.GetCodeString().GetAscii();
        }
    }

    /// Tell the device to stop sending images.
    qDebug() << "Sending AcquisitionStop command to the device";
    lStop->Execute();

    /// Disable streaming on the device
    qDebug() << "Disable streaming on the controller.";
    lDevice->StreamDisable();

    /// Abort all buffers from the stream and dequeue
    qDebug() << "Aborting buffers still in stream";
    lStream->AbortQueuedBuffers();
    while ( lStream->GetQueuedBufferCount() > 0 ) {
        PvBuffer *lBuffer = NULL;
        PvResult lOperationResult;

        lStream->RetrieveBuffer( &lBuffer, &lOperationResult );
    }

    /// free the buffers
    FreeStreamBuffers( &lBufferList );

    /// Close the stream
    qDebug() << "Closing stream";
    lStream->Close();
    PvStream::Free( lStream );
    lStream = NULL;

    /// stop por las dudas poner de nuevo en true
    m_stopThreadRequested = true;
}

} // namespace Z3D
