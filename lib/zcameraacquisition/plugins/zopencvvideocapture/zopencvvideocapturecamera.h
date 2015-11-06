#ifndef Z3D_CAMERAACQUISITION_PLUGIN___OPENCVVIDEOCAPTURECAMERA_H
#define Z3D_CAMERAACQUISITION_PLUGIN___OPENCVVIDEOCAPTURECAMERA_H

#include "zcamerainterface_p.h"

#include <QMutex>

namespace Z3D
{

class OpenCVVideoCaptureCamera : public ZCameraBase
{
    Q_OBJECT

public:
    explicit OpenCVVideoCaptureCamera(int deviceId, QObject *parent = 0);
    ~OpenCVVideoCaptureCamera();

signals:

public slots:
    /// acquisition control
    virtual bool startAcquisition();
    virtual bool stopAcquisition();

    /// camera attributes (settings)
    virtual QList<ZCameraAttribute> getAllAttributes();
    virtual QVariant getAttribute(const QString &deviceID) const;

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify);

    void grabLoop();

private:
    cv::VideoCapture *m_capture;

    bool m_stopThreadRequested;

    QMutex m_mutex;

    QList<QSize> m_frameSizes;

    static QMap<int, QString> m_opencvAttributeNames;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___OPENCVVIDEOCAPTURECAMERA_H
