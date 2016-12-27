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

#include "zpointclouddata.h"

#include <QDebug>

ZPointCloudData::ZPointCloudData(const Z3D::ZSimplePointCloud::Ptr &pointCloud)
    : m_count(0)
{
    const auto &points = pointCloud->points;
    m_data.resize(6 * points.size());

    // calculate center
    Z3D::ZSimplePointCloud::PointType min = points.front(), max = points.front();
    for (const auto &point : points) {
        for (int i=0; i<3; ++i) {
            if (point[i] > max[i])
                max[i] = point[i];
            if (point[i] < min[i])
                min[i] = point[i];
        }
    }
    Z3D::ZSimplePointCloud::PointType cen = (min + max)/2.f;
    qDebug() << "min: (" << min[0] << "," << min[1] << "," << min[2] << ")"
             << "max: (" << max[0] << "," << max[1] << "," << max[2] << ")"
             << "center: (" << cen[0] << "," << cen[1] << "," << cen[2] << ")";

    uint32_t rgb = (static_cast<uint32_t>(255) << 24 |
                    static_cast<uint32_t>(255) << 16 |
                    static_cast<uint32_t>(255) <<  8 |
                    static_cast<uint32_t>(255));
    qDebug() << rgb;

    // add points
    for (const auto &point : points) {
        const uint32_t rgb = *reinterpret_cast<const uint32_t*>(&point[3]);
        uint32_t r = (rgb & 0x00FF0000) >> 16;
        uint32_t g = (rgb & 0x0000FF00) >>  8;
        uint32_t b = (rgb & 0x000000FF) >>  0;
        //qDebug() << r << g << b;

        add(point[0]-cen[0], point[1]-cen[1], point[2]-cen[2],
                float(r)/256.f, float(g)/256.f, float(b)/256.f);//point[3]);
    }
}

void ZPointCloudData::add(const GLfloat &x, const GLfloat &y, const GLfloat &z, const GLfloat &r, const GLfloat &g, const GLfloat &b)
{
    GLfloat *p = m_data.data() + m_count;
    *p++ = 0.01f*x;
    *p++ = 0.01f*y;
    *p++ = 0.01f*z;
    *p++ = r;
    *p++ = g;
    *p++ = b;
    m_count += 6;
}
