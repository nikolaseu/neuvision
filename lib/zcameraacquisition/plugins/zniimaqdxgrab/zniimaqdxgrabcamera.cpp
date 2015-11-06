#include "zniimaqdxgrabcamera.h"

#include "zniimaqdxgrabcamera_p.h"

#include "zcameraimage.h"

#include <QDebug>

namespace Z3D
{

ZCameraInterface::Ptr ZNIIMAQdxGrabCamera::getCameraByName(QString name)
{
    ZNIIMAQdxGrabCameraPrivate *cameraPrivate = ZNIIMAQdxGrabCameraPrivate::getCameraByName(name);

    if (!cameraPrivate)
        return ZCameraInterface::Ptr(0);

    ZNIIMAQdxGrabCamera *camera = new ZNIIMAQdxGrabCamera();

    camera->d_ptr = cameraPrivate;
    camera->m_uuid = cameraPrivate->m_uuid;
    cameraPrivate->q_ptr = camera;

    QObject::connect(cameraPrivate, SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)),
                     camera,        SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)));
    QObject::connect(cameraPrivate, SIGNAL(error(QString)),
                     camera,        SIGNAL(error(QString)));
    QObject::connect(cameraPrivate, SIGNAL(warning(QString)),
                     camera,        SIGNAL(warning(QString)));
    QObject::connect(cameraPrivate, SIGNAL(attributeChanged(QString,QVariant)),
                     camera,        SIGNAL(attributeChanged(QString,QVariant)));

    return ZCameraInterface::Ptr( camera );
}

ZNIIMAQdxGrabCamera::ZNIIMAQdxGrabCamera(QObject *parent) :
    ZCameraBase(parent)
{

}

ZNIIMAQdxGrabCamera::~ZNIIMAQdxGrabCamera()
{
    /// stop acquisition (if running)
    if (isRunning())
        stopAcquisition();

    d_ptr->deleteLater();
}

bool ZNIIMAQdxGrabCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition())
        return false;

    return d_ptr->startAcquisition();
}

bool ZNIIMAQdxGrabCamera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition())
        return false;

    return d_ptr->stopAcquisition();
}

QList<ZCameraInterface::ZCameraAttribute> ZNIIMAQdxGrabCamera::getAllAttributes()
{
    return d_ptr->getAllAttributes();
}

QVariant ZNIIMAQdxGrabCamera::getAttribute(const QString &name) const
{
    return d_ptr->getAttribute(name);
}

bool ZNIIMAQdxGrabCamera::setBufferSize(int bufferSize)
{
    return d_ptr->setBufferSize(bufferSize);
}

bool ZNIIMAQdxGrabCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    return d_ptr->setAttribute(name, value, notify);
}

} // namespace Z3D
