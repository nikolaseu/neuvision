#include "zflycapture2plugin.h"

#include "zflycapture2camera.h"

namespace Z3D
{

QString ZFlyCapture2Plugin::id()
{
    return QString("ZFlyCapture2");
}

QString ZFlyCapture2Plugin::name()
{
    return QString("PGR FlyCapture2 SDK");
}

QString ZFlyCapture2Plugin::version()
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCameraInfo *> ZFlyCapture2Plugin::getConnectedCameras()
{
    //! TODO
    QList<ZCameraInfo *> list;

    return list;
}

ZCameraInterface::Ptr ZFlyCapture2Plugin::getCamera(QVariantMap options)
{
    QList<ZCameraInterface::Ptr> cameraList = ZFlyCapture2Camera::getConnectedCameras();
    if (cameraList.size())
        return cameraList.first();

    return ZCameraInterface::Ptr(0);
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zflycapture2plugin, Z3D::ZFlyCapture2Plugin)
#endif
