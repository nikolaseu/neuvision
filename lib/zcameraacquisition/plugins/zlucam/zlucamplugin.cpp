#include "zlucamplugin.h"

#include "zlucamcamera.h"

#include <QDebug>
#include <QStringList>

namespace Z3D
{

QString ZLuCamPlugin::id()
{
    return QString("ZLuCam");
}

QString ZLuCamPlugin::name()
{
    return QString("Lumenera LuCam SDK");
}

QString ZLuCamPlugin::version()
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCameraInfo *> ZLuCamPlugin::getConnectedCameras()
{
    QList<ZCameraInfo *> camerasList;

    QList<ZCameraInterface::Ptr> cameras =  LuCamCamera::getConnectedCameras();
    foreach (ZCameraInterface::Ptr camera, cameras) {
        QVariantMap extraData;
        extraData["Name"] = camera->uuid();
        camerasList << new ZCameraInfo(this, camera->uuid(), extraData);
    }

    return camerasList;
}

ZCameraInterface::Ptr ZLuCamPlugin::getCamera(QVariantMap options)
{
    QString cameraName = options.value("Name").toString();
    ZCameraInterface::Ptr camera = LuCamCamera::getCameraByName(cameraName);

    if (camera && options.contains("ConfigFile")) {
        QString configFileName = options.value("ConfigFile").toString();
        camera->loadConfiguration(configFileName);
    }

    return camera;
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zlucam, Z3D::ZLuCamPlugin)
#endif // QT_VERSION < 0x050000
