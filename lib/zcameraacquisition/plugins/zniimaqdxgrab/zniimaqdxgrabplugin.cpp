#include "zniimaqdxgrabplugin.h"

#include "zniimaqdxgrabcamera.h"

#include "NIIMAQdx.h"
//#include "nivision.h"

#include <QDebug>
#include <QStringList>

namespace Z3D
{

QString ZNIIMAQdxGrabPlugin::name()
{
    return QString("ZNIIMAQdxGrab");
}

QString ZNIIMAQdxGrabPlugin::version()
{
    return QString(Z3D_VERSION_STR);
}

QStringList ZNIIMAQdxGrabPlugin::getConnectedCameras()
{
    IMAQdxError status = IMAQdxErrorSuccess;
    uInt32 numCameras;
    bool connectedOnly = true;

    QStringList camerasList;

    status = IMAQdxEnumerateCameras(NULL, &numCameras, connectedOnly);
    if (status) {
        qWarning() << "failed to obtain connected cameras information";
        return camerasList;
    }

    if (numCameras > 0) {
        qDebug() << numCameras << "connected cameras found";

        IMAQdxCameraInformation *camerasInfo = new IMAQdxCameraInformation[numCameras];
        status = IMAQdxEnumerateCameras(camerasInfo, &numCameras, connectedOnly);

        if (status) {
            qWarning() << "failed to obtain cameras information";
            return camerasList;
        }

        for (unsigned int iCam=0; iCam<numCameras; ++iCam) {
            qDebug() << "__________________________________________________________";
            qDebug() << "CameraAttributeURL" << camerasInfo[iCam].CameraAttributeURL;
            qDebug() << "CameraFileName    " << camerasInfo[iCam].CameraFileName;
            qDebug() << "InterfaceName     " << camerasInfo[iCam].InterfaceName;
            qDebug() << "ModelName         " << camerasInfo[iCam].ModelName;
            qDebug() << "SerialNumberHi    " << camerasInfo[iCam].SerialNumberHi;
            qDebug() << "SerialNumberLo    " << camerasInfo[iCam].SerialNumberLo;
            qDebug() << "VendorName        " << camerasInfo[iCam].VendorName;
            qDebug() << "Version           " << camerasInfo[iCam].Version;

//            emit cameraConnected(camerasInfo[iCam].InterfaceName);

            camerasList << camerasInfo[iCam].InterfaceName;
        }
    }

    return camerasList;
}

ZCameraInterface::Ptr ZNIIMAQdxGrabPlugin::getCamera(QVariantMap options)
{
    QString cameraName = options.value("Name").toString();
    return ZNIIMAQdxGrabCamera::getCameraByName(cameraName);
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zniimaqdxgrab, Z3D::ZNIIMAQdxGrabPlugin)
#endif // QT_VERSION < 0x050000
