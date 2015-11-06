#ifndef Z3D_CALIBRATEDCAMERA___ZCALIBRATEDCAMERA_H
#define Z3D_CALIBRATEDCAMERA___ZCALIBRATEDCAMERA_H

#include "zcalibratedcamera_global.h"

#include <Z3DCameraInterface>
#include <Z3DCameraCalibration>

namespace Z3D
{

class Z3D_CALIBRATEDCAMERA_SHARED_EXPORT ZCalibratedCamera : public QObject
{
    Q_OBJECT

public:
    typedef QSharedPointer<Z3D::ZCalibratedCamera> Ptr;
    typedef QPointer<Z3D::ZCalibratedCamera> WeakPtr;

    explicit ZCalibratedCamera(ZCameraInterface::Ptr camera, ZCameraCalibration::Ptr cameraCalibration);
    ~ZCalibratedCamera();

    ZCameraInterface::Ptr camera() const { return m_camera; }
    ZCameraCalibration::Ptr calibration() const { return m_cameraCalibration; }

    void setCalibration(Z3D::ZCameraCalibration::Ptr newCalibration);

signals:
    void calibrationChanged(Z3D::ZCameraCalibration::Ptr newCalibration);

protected:
    ZCameraInterface::Ptr m_camera;
    ZCameraCalibration::Ptr m_cameraCalibration;
};

} // namespace Z3D

#endif // Z3D_CALIBRATEDCAMERA___ZCALIBRATEDCAMERA_H
