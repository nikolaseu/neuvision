#pragma once

#include "zcalibratedcamera_global.h"
#include "zcalibratedcamera.h"

#include <QList>

namespace Z3D
{

class Z3D_CALIBRATEDCAMERA_SHARED_EXPORT CalibratedCameraProvider
{
public:
    static QList<Z3D::ZCalibratedCamera::Ptr> loadCameras(QString folder = QString());

    static Z3D::ZCalibratedCamera::Ptr getCalibratedCamera(QSettings *settings);

protected:
    CalibratedCameraProvider();
};

} // namespace Z3D
