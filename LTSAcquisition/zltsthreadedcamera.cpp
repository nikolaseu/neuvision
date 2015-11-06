#include "zltsthreadedcamera.h"

#include <QThread>

ZLTSThreadedCamera::ZLTSThreadedCamera(Z3D::LTSCamera3D::Ptr camera3D, QObject *parent) :
    QObject(parent),
    m_camera3D(camera3D),
    m_cameraThread(new QThread(this)),
    m_camera3DThread(new QThread(this)),
    m_framesRecorder(new Z3D::ZCameraFramesRecorder()),
    m_cloudRecorder(new Z3D::PointCloudRecorder())
{
    m_camera3D->camera2d()->moveToThread(m_cameraThread);
    m_camera3D->moveToThread(m_camera3DThread);

    m_framesRecorder->setCamera(m_camera3D->camera2d()->camera().data());
    m_framesRecorder->moveToThread(m_camera3DThread);

    m_cloudRecorder->setCamera(m_camera3D.data());
    m_cloudRecorder->moveToThread(m_camera3DThread);

    /// scheduled as often as possible. more than highest
    m_cameraThread->start(QThread::TimeCriticalPriority);
    m_camera3DThread->start(QThread::HighestPriority);
}

ZLTSThreadedCamera::~ZLTSThreadedCamera()
{
    m_camera3D->stopAcquisition();

    m_cameraThread->quit();
    if (!m_cameraThread->wait(5000)) {
        m_cameraThread->terminate();
    }

    m_camera3DThread->quit();
    if (!m_camera3DThread->wait(5000)) {
        m_camera3DThread->terminate();
    }
}

Z3D::LTSCamera3D::Ptr ZLTSThreadedCamera::camera3d() const
{
    return m_camera3D;
}

Z3D::ZCalibratedCamera::Ptr ZLTSThreadedCamera::camera2d() const
{
    return m_camera3D->camera2d();
}

void ZLTSThreadedCamera::startAcquisition()
{
    m_camera3D->startAcquisition();
}

void ZLTSThreadedCamera::stopAcquisition()
{
    m_camera3D->stopAcquisition();
}

void ZLTSThreadedCamera::setFrameRecorderEnabled(bool enabled)
{
    m_framesRecorder->setEnabled(enabled);
}

void ZLTSThreadedCamera::setCloudRecorderEnabled(bool enabled)
{
    m_cloudRecorder->setEnabled(enabled);
}
