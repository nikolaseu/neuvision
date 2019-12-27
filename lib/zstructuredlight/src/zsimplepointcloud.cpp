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

#include "ZStructuredLight/zsimplepointcloud.h"

#include <ZPointCloud/zpointfield.h>

namespace Z3D
{

ZSimplePointCloud::ZSimplePointCloud(const ZSimplePointCloud::PointVector points,
                                     QVector<unsigned int> indices,
                                     QObject *parent)
    : ZPointCloud(
        1, // height
        points.size(), // width
        sizeof(PointType), // point step
        sizeof(PointType) * points.size(), // row step
        {
            ZPointField::position(0, ZPointField::FLOAT32, 3),
            ZPointField::normal(12, ZPointField::FLOAT32, 3),
            ZPointField::color(24, ZPointField::UINT8, 4),
            ZPointField::radii(28, ZPointField::FLOAT32, 1)
        },
        parent)
    , m_points(std::move(points))
    , m_indices(std::move(indices))
{

}

ZSimplePointCloud::~ZSimplePointCloud()
{

}

QByteArray ZSimplePointCloud::vertexData() const
{
    /// do not copy data, but we need to be careful!
    return QByteArray::fromRawData(reinterpret_cast<const char*>(m_points.data()),
                                   int(m_points.size() * sizeof(PointType)));
}

QByteArray ZSimplePointCloud::trianglesData() const
{
    /// do not copy data, but we need to be careful!
    return QByteArray::fromRawData(reinterpret_cast<const char*>(m_indices.constData()),
                                   int(sizeof(quint32)) * m_indices.size());
}

} // namespace Z3D
