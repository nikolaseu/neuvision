#ifndef Z3D_CAMERAACQUISITION_PLUGIN___ZNIIMAQDXGRABPLUGIN_H
#define Z3D_CAMERAACQUISITION_PLUGIN___ZNIIMAQDXGRABPLUGIN_H

#include <QObject>

#include "zcamerainterface.h"
#include "zcameraplugininterface.h"

namespace Z3D
{

class ZNIIMAQdxGrabPlugin : public QObject, ZCameraPluginInterface
{
    Q_OBJECT

#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "z3d.cameraacquisition.cameraplugininterface" FILE "zniimaqdxgrab.json")
#endif // QT_VERSION >= 0x050000

    Q_INTERFACES(Z3D::ZCameraPluginInterface)

public:
    /// plugin information
    virtual QString name();
    virtual QString version();

    /// camera utilities
    virtual QStringList getConnectedCameras();
    virtual ZCameraInterface::Ptr getCamera(QVariantMap options);
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___ZNIIMAQDXGRABPLUGIN_H
