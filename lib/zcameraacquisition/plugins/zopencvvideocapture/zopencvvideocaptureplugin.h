#ifndef Z3D_CAMERAACQUISITION_PLUGIN___ZOPENCVVIDEOCAPTUREPLUGIN_H
#define Z3D_CAMERAACQUISITION_PLUGIN___ZOPENCVVIDEOCAPTUREPLUGIN_H

#include <QObject>

#include "zcamerainterface.h"
#include "zcameraplugininterface.h"

namespace Z3D
{

class ZOpenCVVideoCapturePlugin : public QObject, ZCameraPluginInterface
{
    Q_OBJECT

#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "z3d.cameraacquisition.cameraplugininterface" FILE "zopencvvideocapture.json")
#endif // QT_VERSION >= 0x050000

    Q_INTERFACES(Z3D::ZCameraPluginInterface)

public:
    explicit ZOpenCVVideoCapturePlugin();
    virtual ~ZOpenCVVideoCapturePlugin();

    /// plugin information
    virtual QString id();
    virtual QString name();
    virtual QString version();

    /// camera utilities
    virtual QList<ZCameraInfo *> getConnectedCameras();
    virtual ZCameraInterface::Ptr getCamera(QVariantMap options);
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___ZOPENCVVIDEOCAPTUREPLUGIN_H
