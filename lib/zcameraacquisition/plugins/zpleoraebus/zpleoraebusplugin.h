#ifndef Z3D_CAMERAACQUISITION_PLUGIN___ZPLEORAEBUSPLUGIN_H
#define Z3D_CAMERAACQUISITION_PLUGIN___ZPLEORAEBUSPLUGIN_H

#include <QObject>

#include "zcamerainterface.h"
#include "zcameraplugininterface.h"

class PvSystem;

namespace Z3D
{

class ZPleoraeBUSPlugin : public QObject, ZCameraPluginInterface
{
    Q_OBJECT

#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "z3d.cameraacquisition.cameraplugininterface" FILE "zpleoraebus.json")
#endif // QT_VERSION >= 0x050000

    Q_INTERFACES(Z3D::ZCameraPluginInterface)

public:
    /// plugin information
    virtual QString id();
    virtual QString name();
    virtual QString version();

    /// camera utilities
    virtual QList<ZCameraInfo *> getConnectedCameras();
    virtual ZCameraInterface::Ptr getCamera(QVariantMap options);

private:
    static PvSystem *s_pvSystem;
    static PvSystem *getPvSystem();
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___ZPLEORAEBUSPLUGIN_H
