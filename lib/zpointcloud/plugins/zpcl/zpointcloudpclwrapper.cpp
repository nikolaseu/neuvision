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

#include "ZPointCloud/zpointfield.h"

#include <QLoggingCategory>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>

namespace Z3D
{

namespace // anonymous namespace
{
Q_LOGGING_CATEGORY(loggingCategory, "z3d.zpointcloud.plugins.zpcl.zpointcloudpclwrapper")

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

ZPointField *fromPclPointField(pcl::PCLPointField *field)
{
    return new ZPointField(field->name.c_str(), field->offset, fromPCLDatatype(field->datatype), field->count);
}

} // anonymous namespace

ZPointCloudPCLWrapper::ZPointCloudPCLWrapper(pcl::PCLPointCloud2 *pointCloud)
    : ZPointCloud()
    , m_pointCloud(pointCloud)
{
    qDebug(loggingCategory) << "creating point cloud with" << width() * height() << "points,"
             << "size:" << vertexData().size() << "bytes";
}

ZPointCloudPCLWrapper::~ZPointCloudPCLWrapper()
{
    qDebug(loggingCategory) << "destroying" << this;
    delete m_pointCloud;
    for (auto field : m_fields) {
        delete field;
    }
}

void ZPointCloudPCLWrapper::updateAttributes()
{
    for (auto field : m_fields) {
        delete field;
    }
    m_fields.clear();
    m_fields.reserve(m_pointCloud->fields.size());
    for (pcl::PCLPointField &pf : m_pointCloud->fields) {
        m_fields.push_back(fromPclPointField(&pf));
    }
}

unsigned int ZPointCloudPCLWrapper::height() const
{
    return m_pointCloud->height;
}

unsigned int ZPointCloudPCLWrapper::width() const
{
    return m_pointCloud->width;
}

unsigned int ZPointCloudPCLWrapper::pointStep() const
{
    return m_pointCloud->point_step;
}

unsigned int ZPointCloudPCLWrapper::rowStep() const
{
    return m_pointCloud->row_step;
}

QByteArray ZPointCloudPCLWrapper::vertexData() const
{
    /// do not copy data, but we need to be careful!
    return QByteArray::fromRawData(reinterpret_cast<const char*>(&m_pointCloud->data[0]), int(m_pointCloud->data.size()));
}

const std::vector<ZPointField*> &ZPointCloudPCLWrapper::fields() const
{
    return m_fields;
}

} // namespace Z3D
