#ifndef Z3D_CAMERAACQUISITION_PLUGIN___ZPLEORAEBUSCAMERA_H
#define Z3D_CAMERAACQUISITION_PLUGIN___ZPLEORAEBUSCAMERA_H

#include "zcamerainterface_p.h"

namespace Z3D
{

class ZPleoraeBUSCameraPrivate;

class ZPleoraeBUSCamera : public ZCameraBase
{
    Q_OBJECT

public:
    static ZCameraInterface::Ptr getCameraByMAC(QString name);

    explicit ZPleoraeBUSCamera(QObject *parent = 0);
    virtual ~ZPleoraeBUSCamera();

public slots:
    virtual bool startAcquisition();
    virtual bool stopAcquisition();

    virtual QList<ZCameraAttribute> getAllAttributes();
    virtual QVariant getAttribute(const QString &name) const;

    virtual bool setBufferSize(int bufferSize);

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify);

private:
    ZPleoraeBUSCameraPrivate *d_ptr;
    Q_DECLARE_PRIVATE(ZPleoraeBUSCamera)
    Q_DISABLE_COPY(ZPleoraeBUSCamera)
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___ZPLEORAEBUSCAMERA_H
