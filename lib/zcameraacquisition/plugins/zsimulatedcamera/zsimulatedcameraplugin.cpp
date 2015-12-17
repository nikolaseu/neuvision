#include "zsimulatedcameraplugin.h"

#include "zsimulatedcamera.h"

namespace Z3D
{

ZSimulatedCameraPlugin::ZSimulatedCameraPlugin()
{
}

QString ZSimulatedCameraPlugin::id()
{
    return QString("ZSIMULATED");
}

QString ZSimulatedCameraPlugin::name()
{
    return QString("Simulated camera");
}

QString ZSimulatedCameraPlugin::version()
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCameraInfo*> ZSimulatedCameraPlugin::getConnectedCameras()
{
    QList<ZCameraInfo*> list;
    return list;
}

ZCameraInterface::Ptr ZSimulatedCameraPlugin::getCamera(QVariantMap options)
{
    return ZCameraInterface::Ptr( new Z3D::ZSimulatedCamera(options) );
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zsimulatedcamera, Z3D::SimulatedCameraPlugin)
#endif
