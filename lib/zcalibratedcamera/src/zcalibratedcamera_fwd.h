#pragma once

#include <memory>
#include <QPointer>

namespace Z3D
{

class ZCalibratedCamera;

using ZCalibratedCameraPtr = std::shared_ptr<Z3D::ZCalibratedCamera>;
using ZCalibratedCameraWeakPtr = QPointer<Z3D::ZCalibratedCamera>;

} // namespace Z3D

