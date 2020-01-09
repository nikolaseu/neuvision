#pragma once

#include "ZPointCloud/zpointcloud.h"

class QString;

namespace Z3D::ZQtGeometryLoaders
{

ZPointCloudUniquePtr loadPointCloud(const QString &fileName);

} // namespace Z3D::ZQtGeometryLoaders
