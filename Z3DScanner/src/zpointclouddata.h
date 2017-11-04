/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ZPOINTCLOUDDATA_H
#define ZPOINTCLOUDDATA_H

#include "zsimplepointcloud.h"

#include <qopengl.h>
#include <QVector>

class ZPointCloudData
{
public:
    typedef std::shared_ptr<ZPointCloudData> Ptr;

    explicit ZPointCloudData(const Z3D::ZSimplePointCloud::Ptr &pointCloud);
    const GLfloat *constData() const { return m_data.constData(); }
    int count() const { return m_count; }
    int vertexCount() const { return m_count / 6; }

private:
    void add(const GLfloat &x, const GLfloat &y, const GLfloat &z, const GLfloat &r, const GLfloat &g, const GLfloat &b);

    QVector<GLfloat> m_data;
    int m_count;
};

#endif // ZPOINTCLOUDDATA_H
