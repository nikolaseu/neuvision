#ifndef Z3D_CAMERAACQUISITION_PLUGIN___ZQTCAMERA_H
#define Z3D_CAMERAACQUISITION_PLUGIN___ZQTCAMERA_H

#include "zcamerainterface.h"
#include "zcamerainterface_p.h"

#include <QCamera>
#include <QCameraImageCapture>

namespace Z3D
{

class ZQtCamera : public ZCameraBase
{
    Q_OBJECT

public:
    explicit ZQtCamera(QCamera *qcamera);
    ~ZQtCamera();

public slots:
    virtual bool startAcquisition() override;
    virtual bool stopAcquisition() override;

    virtual QList<ZCameraAttribute> getAllAttributes() override;
    virtual bool setAttribute(const QString &name, const QVariant &value) override;
    virtual QVariant getAttribute(const QString &name) const override;

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify) override;

    void onImageCaptured(int id, const QImage & preview);
    void onImageSaved(int id, const QString &fileName);
    void onImageAvailable(int id, const QVideoFrame & buffer);
    void onCaptureError(int id, QCameraImageCapture::Error error, QString message);
    void onReadyForCaptureChanged(bool ready);

private:
    QCamera *m_qcamera;
    QCameraImageCapture *m_qcameraImageCapture;

    bool m_isCapturing;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___ZQTCAMERA_H
