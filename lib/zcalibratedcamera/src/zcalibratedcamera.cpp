#include "zcalibratedcamera.h"

#include <QDebug>
#include <QMetaType>

namespace Z3D
{

static int z3dCalibratedCameraListPtrTypeId = qRegisterMetaType< QList<Z3D::ZCalibratedCamera::Ptr> >("QList<Z3D::ZCalibratedCamera::Ptr>");

ZCalibratedCamera::ZCalibratedCamera(ZCameraInterface::Ptr camera, ZCameraCalibration::Ptr cameraCalibration) :
    m_camera(camera),
    m_cameraCalibration(cameraCalibration)
{

}

ZCalibratedCamera::~ZCalibratedCamera()
{
//    qDebug() << Q_FUNC_INFO;
}

void ZCalibratedCamera::setCalibration(ZCameraCalibration::Ptr newCalibration)
{
    if (m_cameraCalibration == newCalibration)
        return;

    m_cameraCalibration = newCalibration;

    emit calibrationChanged(newCalibration);
}

} // namespace Z3D
