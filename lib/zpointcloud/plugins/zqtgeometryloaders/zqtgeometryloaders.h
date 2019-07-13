#pragma once

#include "zpointcloud_fwd.h"

class QString;

namespace Z3D
{

namespace ZQtGeometryLoaders
{

ZPointCloudPtr loadPointCloud(const QString &fileName);

} // namespace ZQtGeometryLoaders

} // namespace Z3D
