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

#include "zpointcloudpclwrapper.h"

#include "ZCore/zlogging.h"
#include "ZPointCloud/zpointfield.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zpointcloud.zpcl", QtInfoMsg)

namespace Z3D
{

namespace // anonymous namespace
{

ZPointField::PointFieldTypes fromPCLDatatype(const pcl::uint8_t &datatype)
{
    switch (datatype) {
    case pcl::PCLPointField::INT8:
        return ZPointField::INT8;
    case pcl::PCLPointField::UINT8:
        return ZPointField::UINT8;
    case pcl::PCLPointField::INT16:
        return ZPointField::INT16;
    case pcl::PCLPointField::UINT16:
        return ZPointField::UINT16;
    case pcl::PCLPointField::INT32:
        return ZPointField::INT32;
    case pcl::PCLPointField::UINT32:
        return ZPointField::UINT32;
    case pcl::PCLPointField::FLOAT32:
        return ZPointField::FLOAT32;
    case pcl::PCLPointField::FLOAT64:
        return ZPointField::FLOAT64;
    }

    return ZPointField::INT8;
}

ZPointField *fromPclPointField(const pcl::PCLPointField &field)
{
    if ((strcmp(field.name.c_str(), "rgb") == 0 || strcmp(field.name.c_str(), "rgba") == 0)
        && (field.datatype == pcl::PCLPointField::FLOAT32 || field.datatype == pcl::PCLPointField::UINT32))
    {
        // special handling for color since pcl says it's a float32 or uint32
        return ZPointField::color(field.offset, ZPointField::UINT8, 4);
    }

    return new ZPointField(field.name.c_str(),
                           field.offset,
                           fromPCLDatatype(field.datatype),
                           field.count);
}

} // anonymous namespace

ZPointCloudUniquePtr ZPointCloudPCLWrapper::create(pcl::PCLPointCloud2 *pointCloud)
{
    std::vector<ZPointField *> fields;
    fields.reserve(pointCloud->fields.size());
    for (const pcl::PCLPointField &pf : pointCloud->fields) {
        fields.push_back(fromPclPointField(pf));
    }

    return std::make_unique<ZPointCloudPCLWrapper>(pointCloud, fields);
}

ZPointCloudPCLWrapper::ZPointCloudPCLWrapper(pcl::PCLPointCloud2 *pointCloud,
                                             const std::vector<ZPointField *> &fields,
                                             QObject *parent)
    : ZPointCloud(pointCloud->height,
                  pointCloud->width,
                  pointCloud->point_step,
                  pointCloud->row_step,
                  fields,
                  parent)
    , m_pointCloud(pointCloud)
{
    zDebug() << "creating point cloud with" << width() * height() << "points,"
             << "size:" << vertexData().size() << "bytes";
}

ZPointCloudPCLWrapper::~ZPointCloudPCLWrapper()
{
    zDebug() << "destroying" << this;
    delete m_pointCloud;
}

QByteArray ZPointCloudPCLWrapper::vertexData() const
{
    /// do not copy data, but we need to be careful!
    return QByteArray::fromRawData(reinterpret_cast<const char*>(m_pointCloud->data.data()),
                                   int(m_pointCloud->data.size()));
}

QByteArray ZPointCloudPCLWrapper::trianglesData() const
{
    return {};
}

} // namespace Z3D
