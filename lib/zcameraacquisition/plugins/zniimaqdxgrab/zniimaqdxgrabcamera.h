#ifndef Z3D_CAMERAACQUISITION_PLUGIN___ZNIIMAQDXGRABCAMERA_H
#define Z3D_CAMERAACQUISITION_PLUGIN___ZNIIMAQDXGRABCAMERA_H

#include "zcamerainterface_p.h"

namespace Z3D
{

class ZNIIMAQdxGrabCameraPrivate;

class ZNIIMAQdxGrabCamera : public ZCameraBase
{
    Q_OBJECT

public:
    static ZCameraInterface::Ptr getCameraByName(QString name);

    explicit ZNIIMAQdxGrabCamera(QObject *parent = 0);
    ~ZNIIMAQdxGrabCamera();

public slots:
    virtual bool startAcquisition();
    virtual bool stopAcquisition();

    virtual QList<ZCameraAttribute> getAllAttributes();
    virtual QVariant getAttribute(const QString &name) const;

    virtual bool setBufferSize(int bufferSize);

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify);

private:
    ZNIIMAQdxGrabCameraPrivate *d_ptr;
    Q_DECLARE_PRIVATE(ZNIIMAQdxGrabCamera)
    Q_DISABLE_COPY(ZNIIMAQdxGrabCamera)
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___ZNIIMAQDXGRABCAMERA_H
