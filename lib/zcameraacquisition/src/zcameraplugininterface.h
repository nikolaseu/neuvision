#ifndef Z3D_CAMERAACQUISITION___ZCAMERAPLUGININTERFACE_H
#define Z3D_CAMERAACQUISITION___ZCAMERAPLUGININTERFACE_H

#include "zcameraacquisition_global.h"
#include "zcamerainfo.h"
#include "zcamerainterface.h"

#include <QtPlugin>
#include <QList>
#include <QVariantMap>

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraPluginInterface
{
public:
    virtual ~ZCameraPluginInterface() {}

    /// plugin information
    virtual QString id() = 0;
    virtual QString name() = 0;
    virtual QString version() = 0;

    /// camera utilities
    virtual QList<ZCameraInfo *> getConnectedCameras() = 0;
    virtual ZCameraInterface::Ptr getCamera(QVariantMap options) = 0;
};

} // namespace Z3D

#define ZCameraPluginInterface_iid "z3d.cameraacquisition.cameraplugininterface"

Q_DECLARE_INTERFACE(Z3D::ZCameraPluginInterface, ZCameraPluginInterface_iid)

#endif // Z3D_CAMERAACQUISITION___ZCAMERAPLUGININTERFACE_H
