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

#include "zpointcloud.h"

#include "zpointcloud_fwd.h"

#include <pcl/features/normal_3d_omp.h>

#include <QDebug>
#include <QMetaType>
#include <QTime>

static int z3dPointCloudPtrTypeId = qRegisterMetaType<Z3D::ZPointCloudPtr>("Z3D::ZPointCloudPtr");

namespace Z3D
{

int ZPointCloud::m_lastUuid = 0;

ZPointCloud::ZPointCloud()
    : m_pointCloud(new PointCloudPCL)
{
    init();
}

ZPointCloud::ZPointCloud(PointCloudPCLPtr pclPointCloud)
    : m_pointCloud(pclPointCloud)
{
    init();
}

void ZPointCloud::setPclPointCloud(PointCloudPCLPtr pointCloud)
{
    m_pointCloud = pointCloud;
    m_cloudNormals.reset();
    m_surfaceElements.reset();
    m_surfaceMesh.reset();
    m_colorHandler.reset();
}

SurfaceNormalsPtr ZPointCloud::pclNormals()
{
    if (!m_cloudNormals) {
        /// compute normals
        QTime time;
        time.start();

        pcl::PointCloud<pcl::Normal>::Ptr cloudNormalsAux(new pcl::PointCloud<pcl::Normal>);

        /// estimate point normals
        pcl::NormalEstimationOMP<PointType, pcl::Normal> ne;
        ne.setSearchMethod (m_tree);
        ne.setInputCloud (m_pointCloud);
        ne.setKSearch (50);
        ne.compute (*cloudNormalsAux);

        /// update
        m_cloudNormals.swap(cloudNormalsAux);

        qDebug() << "normal estimation finished in" << time.elapsed() << "msecs";

        /// normals changed, surface elements will need to be recalculated when needed
        m_surfaceElements.reset();
    }

    return m_cloudNormals;
}

void ZPointCloud::setPclNormals(SurfaceNormalsPtr normals)
{
    m_cloudNormals = normals;
    m_surfaceElements.reset();
    m_surfaceMesh.reset();
    m_colorHandler.reset();
}

SurfaceElementsPtr ZPointCloud::pclSurfaceElements()
{
    if (!m_surfaceElements) {
        QTime time;
        time.start();

        SurfaceElementsPtr surfaceElements = SurfaceElementsPtr(new SurfaceElements);
        /// Concatenate the XYZ and normal fields*
        pcl::concatenateFields(*pclPointCloud(), *pclNormals(), *surfaceElements);

        m_surfaceElements = surfaceElements;

        qDebug() << "point+normal concatenation finished in" << time.elapsed() << "msecs";
    }

    return m_surfaceElements;
}

void ZPointCloud::setPclSurfaceElements(SurfaceElementsPtr surfaceElements)
{
    m_surfaceElements = surfaceElements;
}

pcl::PolygonMesh::Ptr ZPointCloud::pclSurfaceMesh()
{
    if (!m_surfaceMesh) {
        m_surfaceMesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh);
    }

    return m_surfaceMesh;
}

pcl::visualization::PointCloudColorHandlerGenericField<PointType>::Ptr ZPointCloud::colorHandler()
{
    if (!m_colorHandler) {
        m_colorHandler = pcl::visualization::PointCloudColorHandlerGenericField<PointType>::Ptr(
                    new pcl::visualization::PointCloudColorHandlerGenericField<PointType>(m_pointCloud, "intensity"));
    }

    return m_colorHandler;
}

pcl::search::KdTree<PointType>::Ptr ZPointCloud::tree()
{
    return m_tree;
}

void ZPointCloud::init()
{
    m_number = 0;

    m_uuid = ++m_lastUuid;

    m_id = QString("cloud_%1").arg(m_uuid);

    m_tree = pcl::search::KdTree<PointType>::Ptr(new pcl::search::KdTree<PointType>);
}

} // namespace Z3D
