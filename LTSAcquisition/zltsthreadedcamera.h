#ifndef ZLTSTHREADEDCAMERA_H
#define ZLTSTHREADEDCAMERA_H

#include <QObject>

#include <Z3DCameraAcquisition>
#include <Z3DLaserTriangulation>

class QThread;

class ZLTSThreadedCamera : public QObject
{
    Q_OBJECT

public:
    typedef QSharedPointer<ZLTSThreadedCamera> Ptr;

    explicit ZLTSThreadedCamera(Z3D::LTSCamera3D::Ptr camera3D, QObject *parent = 0);
    ~ZLTSThreadedCamera();

    Z3D::LTSCamera3D::Ptr camera3d() const;
    Z3D::ZCalibratedCamera::Ptr camera2d() const;

signals:

public slots:
    void startAcquisition();
    void stopAcquisition();

    void setFrameRecorderEnabled(bool enabled);
    void setCloudRecorderEnabled(bool enabled);

protected:
    Z3D::LTSCamera3D::Ptr m_camera3D;

    QThread *m_cameraThread;
    QThread *m_camera3DThread;

    Z3D::ZCameraFramesRecorder::Ptr m_framesRecorder;
    Z3D::PointCloudRecorder::Ptr m_cloudRecorder;
};

#endif // ZLTSTHREADEDCAMERA_H
