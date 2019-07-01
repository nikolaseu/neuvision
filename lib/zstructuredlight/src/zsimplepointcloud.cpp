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

#include "zsimplepointcloud.h"

#include <zpointfield.h>

namespace Z3D
{

ZSimplePointCloud::ZSimplePointCloud(const ZSimplePointCloud::PointVector points, QObject *parent)
    : ZPointCloud(parent)
    , m_fields({
               new ZPointField("x",    0, ZPointField::FLOAT32, 1),
               new ZPointField("y",    4, ZPointField::FLOAT32, 1),
               new ZPointField("z",    8, ZPointField::FLOAT32, 1),
               new ZPointField("rgb", 12, ZPointField::FLOAT32, 1)
    })
    , m_points(std::move(points))
    , m_width(m_points.size())
    , m_height(1)
{
    PointType min = points.front()
            , max = points.front();

    for (const auto &point : points) {
        for (int i=0; i<3; ++i) {
            if (point[i] > max[i]) {
                max[i] = point[i];
            }
            if (point[i] < min[i]) {
                min[i] = point[i];
            }
        }
    }

    m_minimum = QVector3D(min[0], min[1], min[2]);
    m_maximum = QVector3D(max[0], max[1], max[2]);
}

ZSimplePointCloud::~ZSimplePointCloud()
{
    for (auto field : m_fields) {
        delete field;
    }
}

void ZSimplePointCloud::updateAttributes()
{
    /// nothing to do, we don't have dynamic fields
}

unsigned int ZSimplePointCloud::height() const
{
    return m_height;
}

unsigned int ZSimplePointCloud::width() const
{
    return m_width;
}

unsigned int ZSimplePointCloud::pointStep() const
{
    return sizeof(cv::Vec4f);
}

unsigned int ZSimplePointCloud::rowStep() const
{
    return width();
}

QByteArray ZSimplePointCloud::data() const
{
    /// do not copy data, but we need to be careful!
    return QByteArray::fromRawData(reinterpret_cast<const char*>(m_points.data()), int(m_points.size() * pointStep()));
}

const std::vector<ZPointField *> &ZSimplePointCloud::fields() const
{
    return m_fields;
}

QVector3D ZSimplePointCloud::minimum() const
{
    return m_minimum;
}

QVector3D ZSimplePointCloud::maximum() const
{
    return m_maximum;
}

QVector3D ZSimplePointCloud::center() const
{
    return (m_minimum + m_maximum) / 2.;
}

} // namespace Z3D
