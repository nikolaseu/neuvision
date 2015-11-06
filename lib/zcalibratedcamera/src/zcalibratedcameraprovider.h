#ifndef CALIBRATEDCAMERAPROVIDER_H
#define CALIBRATEDCAMERAPROVIDER_H

#include "zcalibratedcamera_global.h"
#include "zcalibratedcamera.h"

#include <QList>

namespace Z3D
{

class Z3D_CALIBRATEDCAMERA_SHARED_EXPORT CalibratedCameraProvider
{
public:
    static QList<Z3D::ZCalibratedCamera::Ptr> loadCameras(QString folder = QString());

protected:
    CalibratedCameraProvider();
};

} // namespace Z3D

#endif // CALIBRATEDCAMERAPROVIDER_H
