#ifndef Z3D_LASERTRIANGULATION___CAMERA3DINTERFACE_H
#define Z3D_LASERTRIANGULATION___CAMERA3DINTERFACE_H

#include <Z3DCalibratedCamera>
#include <Z3DCameraAcquisition>
#include <Z3DPointCloud>

#include <QObject>
#include <QPointer>
#include <QSharedPointer>

namespace Z3D
{

class Camera3DInterface : public QObject
{
    Q_OBJECT

public:
    typedef QSharedPointer<Z3D::Camera3DInterface> Ptr;
    typedef QPointer<Z3D::Camera3DInterface> WeakPtr;

    explicit Camera3DInterface(Z3D::ZCalibratedCamera::Ptr camera, QObject *parent = 0) :
        QObject(parent),
        m_camera(camera)
    {
        QObject::connect(m_camera->camera().data(), SIGNAL(acquisitionStarted()),
                         this, SIGNAL(acquisitionStarted()));
        QObject::connect(m_camera->camera().data(), SIGNAL(acquisitionStopped()),
                         this, SIGNAL(acquisitionStopped()));
    }

    QString uuid() { return m_camera->camera()->uuid(); }

signals:
    void newImageReceived(Z3D::ZImageGrayscale::Ptr image);
    void newPointCloudReceived(Z3D::ZPointCloud::Ptr cloud);

    void acquisitionStarted();
    void acquisitionStopped();

public slots:
    virtual void startAcquisition() = 0;
    virtual void stopAcquisition() = 0;

    Z3D::ZCalibratedCamera::Ptr camera2d() const { return m_camera; }

protected:
    Z3D::ZCalibratedCamera::Ptr m_camera;
};

} // namespace Z3D

#endif // Z3D_LASERTRIANGULATION___CAMERA3DINTERFACE_H
