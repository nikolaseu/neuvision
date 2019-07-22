#pragma once

#include "ZPointCloud/zpointcloud_fwd.h"
#include "ZPointCloud/zpointcloud_global.h"

class QString;

namespace Z3D
{

namespace ZQtGeometryLoaders
{

ZPointCloudPtr Z3D_ZPOINTCLOUD_SHARED_EXPORT loadPointCloud(const QString &fileName);

} // namespace ZQtGeometryLoaders

} // namespace Z3D
