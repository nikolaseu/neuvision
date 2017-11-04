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

#pragma once

#include "zpointcloud_global.h"
#include "zpointcloud_typedefs.h"

#include "pcl/PolygonMesh.h"
#include "pcl/search/kdtree.h"
#include "pcl/visualization/point_cloud_handlers.h"

#include <QString>

namespace Z3D
{

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZPointCloud
{
public:
    typedef std::shared_ptr<Z3D::ZPointCloud> Ptr;

    explicit ZPointCloud();
    ZPointCloud(PointCloudPCLPtr pclPointCloud);

    inline long number() const { return m_number; }
    inline void setNumber(long number) { m_number = number; }

    QString id() { return m_id; }
    void setId(QString id) { m_id = id; }

    inline PointCloudPCLPtr pclPointCloud() const { return m_pointCloud; }
    void setPclPointCloud(PointCloudPCLPtr pointCloud);

    SurfaceNormalsPtr pclNormals();
    void setPclNormals(SurfaceNormalsPtr normals);

    SurfaceElementsPtr pclSurfaceElements();
    void setPclSurfaceElements(SurfaceElementsPtr surfaceElements);

    pcl::PolygonMesh::Ptr pclSurfaceMesh();

    pcl::visualization::PointCloudColorHandlerGenericField<PointType>::Ptr colorHandler();

    pcl::search::KdTree<PointType>::Ptr tree();

private:
    void init();

    long m_number;

    PointCloudPCLPtr m_pointCloud;
    SurfaceNormalsPtr m_cloudNormals;
    SurfaceElementsPtr m_surfaceElements;
    pcl::PolygonMesh::Ptr m_surfaceMesh;

    pcl::visualization::PointCloudColorHandlerGenericField<PointType>::Ptr m_colorHandler;

    pcl::search::KdTree<PointType>::Ptr m_tree;

    int m_uuid;
    QString m_id;

    static int m_lastUuid;
};

} // namespace Z3D
