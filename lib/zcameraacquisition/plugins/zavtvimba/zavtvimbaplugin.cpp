#include "zavtvimbaplugin.h"
#include "zavtvimbacamera.h"

#include "VimbaCPP/Include/VimbaSystem.h"

#include <QDebug>

namespace Z3D
{

ZAVTVimbaPlugin::ZAVTVimbaPlugin()
{
    /// initialize vimba system
    VmbErrorType err = AVT::VmbAPI::VimbaSystem::GetInstance().Startup();
    if ( VmbErrorSuccess != err )
        qWarning() << "AVT::VmbAPI::VimbaSystem::GetInstance().Startup() failed. Error:" << err;
}

ZAVTVimbaPlugin::~ZAVTVimbaPlugin()
{
    /// shutdown vimba system
    AVT::VmbAPI::VimbaSystem::GetInstance().Shutdown();
}

QString ZAVTVimbaPlugin::id()
{
    return QString("ZAVTVimba");
}

QString ZAVTVimbaPlugin::name()
{
    return QString("AVT Vimba SDK");
}

QString ZAVTVimbaPlugin::version()
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCameraInfo *> ZAVTVimbaPlugin::getConnectedCameras()
{
    QList<ZCameraInfo*> camerasList;

    AVT::VmbAPI::CameraPtrVector cameras;
    /// Get all known cameras
    if ( VmbErrorSuccess == AVT::VmbAPI::VimbaSystem::GetInstance().GetCameras( cameras )) {
        /// reserve space
        camerasList.reserve(cameras.size());

        /// And query the camera details
        for (AVT::VmbAPI::CameraPtrVector::const_iterator iter = cameras.begin(); iter != cameras.end(); ++iter) {
            AVTVimbaCamera camera(*iter);

            QVariantMap extraData;
            extraData["DeviceID"] = camera.deviceID();

            //! FIXME use a friendlyier name for the camera instead of the DeviceID
            camerasList.push_back( new ZCameraInfo(this, camera.deviceID(), extraData) );
        }
    } else {
        qWarning() << "Unable to obtain cameras. AVT::VmbAPI::VimbaSystem::GetInstance().GetCameras() failed.";
    }

    return camerasList;
}

ZCameraInterface::Ptr ZAVTVimbaPlugin::getCamera(QVariantMap options)
{
    QString cameraDeviceID = options.value("DeviceID").toString();

    AVTVimbaCamera *avtCamera = 0;

    AVT::VmbAPI::CameraPtrVector cameras;
    /// Get all known cameras
    if ( VmbErrorSuccess == AVT::VmbAPI::VimbaSystem::GetInstance().GetCameras( cameras )) {
        /// query the camera details
        for (AVT::VmbAPI::CameraPtrVector::const_iterator iter = cameras.begin(); iter != cameras.end(); ++iter) {
            avtCamera = new AVTVimbaCamera(*iter);

            qDebug() << "camera:" << avtCamera->deviceID();

            if (avtCamera->deviceID() == cameraDeviceID)
                break;

            delete avtCamera;
            avtCamera = 0;
        }
    } else {
        qWarning() << "Unable to obtain cameras. AVT::VmbAPI::VimbaSystem::GetInstance().GetCameras() failed.";
    }

    if (avtCamera) {
        return ZCameraInterface::Ptr(avtCamera);
    }

    return ZCameraInterface::Ptr(0);
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zavtvimba, Z3D::ZAVTVimbaPlugin)
#endif
