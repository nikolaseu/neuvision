//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "zpointcloudlibraryplugin.h"

#include "zpointcloudpclwrapper.h"

#include "ZCore/zlogging.h"

#include <pcl/io/pcd_io.h>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zpointcloud.zpcl", QtInfoMsg)

namespace Z3D
{

ZPointCloudLibraryPlugin::ZPointCloudLibraryPlugin(QObject *parent)
    : QObject(parent)
{

}

std::vector<ZPointCloudPluginInterface::Formats> ZPointCloudLibraryPlugin::formats() const
{
    return {
        Formats::PCD
    };
}

ZPointCloudUniquePtr ZPointCloudLibraryPlugin::loadPointCloud(const QString &filename) const
{
    zDebug() << "Trying to read PointCloud from" << filename;

    auto pc = new pcl::PCLPointCloud2();
    if (0 == pcl::io::loadPCDFile(filename.toStdString(), *pc)) {
        return ZPointCloudPCLWrapper::create(pc);
    }

    zWarning() << "Failed to read PointCloud. Invalid/unrecognized format:" << filename;
    delete pc;
    return nullptr;
}

} // namespace Z3D
