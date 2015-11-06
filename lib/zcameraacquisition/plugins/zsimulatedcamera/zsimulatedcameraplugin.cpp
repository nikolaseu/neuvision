#include "zsimulatedcameraplugin.h"

#include "zsimulatedcamera.h"

namespace Z3D
{

SimulatedCameraPlugin::SimulatedCameraPlugin()
{
}

QString SimulatedCameraPlugin::id()
{
    return QString("ZSIMULATED");
}

QString SimulatedCameraPlugin::name()
{
    return QString("Simulated camera");
}

QString SimulatedCameraPlugin::version()
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCameraInfo*> SimulatedCameraPlugin::getConnectedCameras()
{
    QList<ZCameraInfo*> list;
    return list;
}

ZCameraInterface::Ptr SimulatedCameraPlugin::getCamera(QVariantMap options)
{
    return ZCameraInterface::Ptr( new Z3D::SimulatedCamera(options) );
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zsimulatedcamera, Z3D::SimulatedCameraPlugin)
#endif
