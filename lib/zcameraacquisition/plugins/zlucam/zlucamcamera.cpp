#include "zlucamcamera.h"

#include <QDebug>

namespace Z3D
{

LuCamCamera::LuCamCamera(HANDLE cameraHandle, QObject *parent)
    : ZCameraBase(parent)
    , m_cameraHandle(cameraHandle)
{
    /// get camera information (version, serialNumber, etc)
    if (!LucamQueryVersion(m_cameraHandle, &m_lucamVersion)) {
        qCritical() << "[LuCam] QueryVersion failed. Error:" << LucamGetLastErrorForCamera(m_cameraHandle);
    } else {
        qDebug() << "[LuCam]"
                 << "api:" << m_lucamVersion.api
                 << "driver:" << m_lucamVersion.driver
                 << "firmware:" << m_lucamVersion.firmware
                 << "fpga:" << m_lucamVersion.fpga
                 << "serialnumber:" << m_lucamVersion.serialnumber;
    }

    m_uuid = QString("LUMENERA-SN%1").arg(m_lucamVersion.serialnumber);

    if (!LucamCameraReset(m_cameraHandle)) {
        qCritical() << "[LuCam] CameraReset failed. Error:" << LucamGetLastErrorForCamera(m_cameraHandle);
    }

    /**
        LUCAM_FRAME_FORMAT Structure

        ULONG xOffset; // x coordinate on imager of top left corner of subwindow in pixels
        ULONG yOffset; // y coordinate on imager of top left corner of subwindow in pixels
        ULONG width; // width in pixels of subwindow
        ULONG height; // height in pixels of subwindow
        ULONG pixelFormat; // pixel format for data
        union {
            USHORT subSampleX; // sub-sample ratio in x direction in pixels (x:1)
            USHORT binningX; // binning ratio in x direction in pixels (x:1)
        }
        USHORT flagsX; // binning flag for x direction
        union {
            USHORT subSampleY; // sub-sample ratio in y direction in pixels (y:1)
            USHORT binningY; // binning ratio in y direction in pixels (y:1)
        }
        USHORT flagsY; // binning flag for y direction
      */

    FLOAT frameRate;
    if (LucamGetFormat(m_cameraHandle, &m_frameFormat, &frameRate)) {
        qDebug() << "[LuCam]"
                 << "frameRate:" << frameRate
                 << "xOffset:" << m_frameFormat.xOffset
                 << "yOffset:" << m_frameFormat.yOffset
                 << "width:" << m_frameFormat.width
                 << "height:" << m_frameFormat.height
                 << "pixelFormat:" << m_frameFormat.pixelFormat;

    } else {
        qCritical() << "[LuCam] GetFormat failed. Error:" << LucamGetLastErrorForCamera(m_cameraHandle);
    }


    /**
        LUCAM_SNAPSHOT Structure

        FLOAT exposure; // Exposure in milliseconds
        FLOAT gain; // Overall gain as a multiplicative factor
        union {
            struct {
                FLOAT gainRed; // Gain for Red pixels as multiplicative factor
                FLOAT gainBlue; // Gain for Blue pixels as multiplicative factor
                FLOAT gainGrn1; // Gain for Green pixels on Red rows as multiplicative factor
                FLOAT gainGrn2; // Gain for Green pixels on Blue rows as multiplicative factor
            }
            struct {
                FLOAT gainMag; // Gain for Magenta pixels as multiplicative factor
                FLOAT gainCyan; // Gain for Cyan pixels as multiplicative factor
                FLOAT gainYel1; // Gain for Yellow pixels on Magenta rows as multiplicative factor
                FLOAT gainYel2; // Gain for Yellow pixels on Cyan rows as multiplicative factor
            }
        }
        union {
            BOOL useStrobe; // use a flash (backward compatibility)
            ULONG strobeFlags; // use LUCAM_PROP_FLAG_USE and/or
            LUCAM_PROP_FLAG_STROBE_FROM_START_OF_EXPOSURE
        }
        FLOAT strobeDelay; // time interval from when exposure starts to time the flash is fired in milliseconds
        BOOL useHwTrigger; // wait for hardware trigger flag
        FLOAT timeout; // maximum time to wait for hardware trigger prior to returning from function in milliseconds
        LUCAM_FRAME_FORMAT format; // frame format for data
        ULONG shutterType; // Shutter mode of the camera
        FLOAT exposureDelay; // time interval from when the trigger occurs to when the exposure starts
        union {
            ULONG ulReserved1; // (backwards compatibility)
            BOOL bufferlastframe; // set to TRUE if you want TakeFastFrame to return an already received frame
        };
        ULONG ulReserved2; // Reserved for future use (Must be set to zero)
        FLOAT flReserved1; // Reserved for future use (Must be set to zero)
        FLOAT flReserved2; // Reserved for future use (Must be set to zero)
     */

    m_snapshotSettings.format = m_frameFormat;

    m_snapshotSettings.exposure = 30;
    m_snapshotSettings.gain = 1;

    m_snapshotSettings.timeout = 1000.0;
    m_snapshotSettings.gainRed = 1.0;
    m_snapshotSettings.gainBlue = 1.0;
    m_snapshotSettings.gainGrn1 = 1.0;
    m_snapshotSettings.gainGrn2 = 1.0;

    m_snapshotSettings.useStrobe = false;
    m_snapshotSettings.strobeDelay = 0.0;
    m_snapshotSettings.useHwTrigger = false;
    m_snapshotSettings.shutterType = LUCAM_SHUTTER_TYPE_GLOBAL;
    m_snapshotSettings.exposureDelay = 0.0;
    m_snapshotSettings.bufferlastframe = 0;


    /*if (LucamEnableFastFrames(m_cameraHandle, &m_snapshotSettings)) {

    } else {
        qWarning() << "[LuCam] LucamEnableFastFrames error code:" << LucamGetLastErrorForCamera(m_cameraHandle);
    }*/

    m_streamingCallbackId = LucamAddStreamingCallback(m_cameraHandle, &LuCamCamera::imageEventCallback, this);
}

QList<ZCameraInterface::Ptr> LuCamCamera::getConnectedCameras()
{
    ULONG ncam = LucamNumCameras();

    QList<ZCameraInterface::Ptr> cameras;
    cameras.reserve(ncam);

    for (ULONG iCam = 1; iCam <= ncam; ++iCam) {
        //HANDLE LucamCameraOpen(ULONG cameraNumber);
        HANDLE cameraHandle = LucamCameraOpen(iCam);
        if (!cameraHandle) {
            qCritical() << "[LuCam] Can't connect to Lumenera Camera" << iCam;
            continue;
        }

        cameras.push_back(ZCameraInterface::Ptr(new LuCamCamera(cameraHandle)));
    }

    return cameras;
}

ZCameraInterface::Ptr LuCamCamera::getCameraByName(QString name)
{
    QList<ZCameraInterface::Ptr> cameraList = getConnectedCameras();
    if (cameraList.size())
        return cameraList.first();

    return ZCameraInterface::Ptr(0);
}

LuCamCamera::~LuCamCamera()
{
    if (m_cameraHandle) {
        LucamRemoveStreamingCallback(m_cameraHandle, m_streamingCallbackId);

        LucamCameraClose(m_cameraHandle);
    }
}

bool LuCamCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition())
        return false;

    qDebug() << Q_FUNC_INFO;

    //LucamStreamVideoControl(HANDLE hCamera, ULONG controlType, HWND hWnd);
    LucamStreamVideoControl(m_cameraHandle, START_STREAMING, 0);

    //LucamSetTriggerMode(m_cameraHandle, false);
    //m_snapshotCallbackId = LucamAddSnapshotCallback(m_cameraHandle, &LuCamCamera::imageEventCallback, this);

    /// update snapshot settings
    //LONG pFlags;
    //LucamGetProperty(m_cameraHandle, LUCAM_PROP_EXPOSURE, &m_snapshotSettings.exposure, &pFlags);
    //LucamGetProperty(m_cameraHandle, LUCAM_PROP_GAIN, &m_snapshotSettings.gain, &pFlags);

    //LucamEnableFastFrames(m_cameraHandle, &m_snapshotSettings);
    //LucamTriggerFastFrame(m_cameraHandle);

    return true;
}

bool LuCamCamera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition())
        return false;

    qDebug() << Q_FUNC_INFO;
    /*
    LucamRemoveSnapshotCallback(m_cameraHandle, m_snapshotCallbackId);
    LucamDisableFastFrames(m_cameraHandle);
    */

    LucamStreamVideoControl(m_cameraHandle, STOP_STREAMING, 0);

    return true;
}

QList<ZCameraInterface::ZCameraAttribute> LuCamCamera::getAllAttributes()
{
    //! TODO
    return QList<ZCameraInterface::ZCameraAttribute>();
}

QVariant LuCamCamera::getAttribute(const QString &name) const
{
    //! TODO
    return QVariant("INVALID");
}

void LuCamCamera::showSettingsDialog()
{
    //! TODO
    LucamDisplayPropertyPage(m_cameraHandle, 0);

    //LucamDisplayVideoFormatPage(m_cameraHandle, 0);
}

bool LuCamCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    //! TODO
    return false;
}

void __stdcall LuCamCamera::imageEventCallback(void *pCallbackData, BYTE *data, ULONG dataLenght)
{
    return reinterpret_cast<LuCamCamera*>(pCallbackData)->imageEventCallbackInternal(data, dataLenght);
}

void LuCamCamera::imageEventCallbackInternal(BYTE *data, ULONG dataLenght)
{
    LucamTriggerFastFrame(m_cameraHandle);

    const int bytesPerPixel = 1;
/*
    /// get image from buffer
    ImageGrayscale::Ptr &currentImage = getNextBufferImage();

    /// if null or different size
    if (currentImage.isNull() ||
            currentImage->width() != m_frameFormat.width ||
            currentImage->height() != m_frameFormat.height ||
            currentImage->bytesPerPixel() != bytesPerPixel) {
        currentImage.swap( ImageGrayscale::Ptr(new ImageGrayscale(m_frameFormat.width, m_frameFormat.height,
                                                m_frameFormat.xOffset, m_frameFormat.yOffset,
                                                bytesPerPixel)) );
    }
*/

    /// get image from buffer
    ZImageGrayscale::Ptr currentImage = getNextBufferImage(m_frameFormat.width, m_frameFormat.height,
                                                          m_frameFormat.xOffset, m_frameFormat.yOffset,
                                                          bytesPerPixel);

    /// copy data
    currentImage->setBuffer(data);

    /// set image number
    currentImage->setNumber(1);

    /// notify
    emit newImageReceived(currentImage);
}

} // namespace Z3D
