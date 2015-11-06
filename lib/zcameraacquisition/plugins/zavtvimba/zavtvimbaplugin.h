#ifndef Z3D_CAMERAACQUISITION_PLUGIN___AVTVIMBAPLUGIN_H
#define Z3D_CAMERAACQUISITION_PLUGIN___AVTVIMBAPLUGIN_H

#include <QObject>

#include "zcamerainterface.h"
#include "zcameraplugininterface.h"

namespace Z3D
{

class ZAVTVimbaPlugin : public QObject, ZCameraPluginInterface
{
    Q_OBJECT

#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "z3d.cameraacquisition.cameraplugininterface" FILE "zavtvimba.json")
#endif // QT_VERSION >= 0x050000

    Q_INTERFACES(Z3D::ZCameraPluginInterface)

public:
    /// constructor and destructor used to startup/shutdown vimba system
    explicit ZAVTVimbaPlugin();
    virtual ~ZAVTVimbaPlugin();

    /// plugin information
    virtual QString id();
    virtual QString name();
    virtual QString version();

    /// camera utilities
    virtual QList<ZCameraInfo *> getConnectedCameras();
    virtual ZCameraInterface::Ptr getCamera(QVariantMap options);
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___AVTVIMBAPLUGIN_H
