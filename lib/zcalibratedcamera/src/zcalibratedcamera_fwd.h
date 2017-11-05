#pragma once

#include <memory>
#include <QPointer>

namespace Z3D
{

class ZCalibratedCamera;

typedef std::shared_ptr<Z3D::ZCalibratedCamera> ZCalibratedCameraPtr;
typedef QPointer<Z3D::ZCalibratedCamera> ZCalibratedCameraWeakPtr;

} // namespace Z3D

