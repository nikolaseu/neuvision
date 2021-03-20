#pragma once

#include <QtCore/QMetaType>
#include <memory>

namespace Z3D
{

class ZPointCloud;
using ZPointCloudPtr = std::shared_ptr<Z3D::ZPointCloud>;
using ZPointCloudUniquePtr = std::unique_ptr<Z3D::ZPointCloud>;

class ZPointCloudListModel;

} // namespace Z3D

Q_DECLARE_OPAQUE_POINTER(Z3D::ZPointCloudListModel*);
