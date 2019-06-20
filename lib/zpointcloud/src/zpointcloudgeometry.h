/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2018 Nicolas Ulrich <nikolaseu@gmail.com>
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

#pragma once

#include "zpointcloud_global.h"

#include <Qt3DRender/QGeometry>

namespace Z3D
{

class ZPointCloud;
class QPointCloudGeometryPrivate;

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZPointCloudGeometry : public Qt3DRender::QGeometry
{
    Q_OBJECT
    Q_PROPERTY(Z3D::ZPointCloud *pointCloud READ pointCloud WRITE setPointCloud NOTIFY pointCloudChanged)

public:
    explicit ZPointCloudGeometry(QNode *parent = nullptr);
    ~ZPointCloudGeometry() override;

    void updateVertices();

    ZPointCloud *pointCloud() const;

signals:
    void pointCloudChanged(ZPointCloud *pointCloud);

public slots:
    void setPointCloud(ZPointCloud *pointCloud);

private slots:
    void updateAttributes();

private:
    QPointCloudGeometryPrivate *m_p;
};

} // namespace Z3D
