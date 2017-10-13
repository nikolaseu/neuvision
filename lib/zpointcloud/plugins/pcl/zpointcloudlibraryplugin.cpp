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

#include <QDebug>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace Z3D
{

ZPointCloudLibraryPlugin::ZPointCloudLibraryPlugin(QObject *parent)
    : QObject(parent)
{

}

ZPointCloudPtr ZPointCloudLibraryPlugin::loadPointCloud(const QString &filename) const
{
    qDebug() << "Trying to read PointCloud from" << filename;

    if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
        pcl::PCDReader reader;
        auto pc = new pcl::PCLPointCloud2();
        reader.read(filename.toStdString(), *pc);
        return std::make_shared<ZPointCloudPCLWrapper>(pc);
    }
    else if (filename.endsWith(".ply", Qt::CaseInsensitive)) {
        pcl::PLYReader reader;
        auto pc = new pcl::PCLPointCloud2();
        reader.read(filename.toStdString(), *pc);
        return std::make_shared<ZPointCloudPCLWrapper>(pc);
    }

    qWarning() << "Failed to read PointCloud. Invalid/unrecognized format:" << filename;
    return nullptr;
}

} // namespace Z3D
