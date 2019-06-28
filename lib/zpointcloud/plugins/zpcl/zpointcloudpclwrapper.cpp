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

#include "zpointfield.h"

#include <QLoggingCategory>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>

namespace Z3D
{

namespace // anonymous namespace
{
Q_LOGGING_CATEGORY(loggingCategory, "z3d.zpointcloud.plugins.zpcl.zpointcloudpclwrapper")
} // anonymous namespace

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

ZPointField *fromPclPointField(pcl::PCLPointField *field, QObject *parent = nullptr)
{
    return new ZPointField(field->name.c_str(), field->offset, fromPCLDatatype(field->datatype), field->count, parent);
}

ZPointCloudPCLWrapper::ZPointCloudPCLWrapper(pcl::PCLPointCloud2 *pointCloud)
    : ZPointCloud()
    , m_pointCloud(pointCloud)
{
    qDebug(loggingCategory) << "creating point cloud with" << width() * height() << "points,"
             << "size:" << data().size() << "bytes";

    /// update boundary
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromPCLPointCloud2(*m_pointCloud, pc);
    pcl::getMinMax3D(pc, min, max);
    m_minimum = QVector3D(min.x, min.y, min.z);
    m_maximum = QVector3D(max.x, max.y, max.z);

    /// update center
    m_center = m_minimum + (m_maximum - m_minimum)/2.0;
}

ZPointCloudPCLWrapper::~ZPointCloudPCLWrapper()
{
    delete m_pointCloud;
}

void ZPointCloudPCLWrapper::updateAttributes()
{
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

QByteArray ZPointCloudPCLWrapper::data() const
{
    // return QByteArray(reinterpret_cast<char*>(&m_pointcloud->data[0]), int(m_pointcloud->data.size()));

    /// do not copy data, but we need to be careful!
    return QByteArray::fromRawData(reinterpret_cast<const char*>(&m_pointCloud->data[0]), int(m_pointCloud->data.size()));
}

const std::vector<ZPointField *> &ZPointCloudPCLWrapper::fields() const
{
    return m_fields;
}

QVector3D ZPointCloudPCLWrapper::minimum() const
{
    return m_minimum;
}

QVector3D ZPointCloudPCLWrapper::maximum() const
{
    return m_maximum;
}

QVector3D ZPointCloudPCLWrapper::center() const
{
    return m_center;
}

} // namespace Z3D
