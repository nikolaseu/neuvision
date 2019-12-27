#pragma once

#include <memory>

namespace Z3D
{

class ZPointCloud;
using ZPointCloudPtr = std::shared_ptr<Z3D::ZPointCloud>;
using ZPointCloudUniquePtr = std::unique_ptr<Z3D::ZPointCloud>;

class ZPointCloudListModel;

} // namespace Z3D
