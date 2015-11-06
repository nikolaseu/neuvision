#include "zpointcloud.h"

#include "pcl/features/normal_3d_omp.h"

#include <QDebug>
#include <QMetaType>
#include <QTime>

static int z3dPointCloudPtrTypeId = qRegisterMetaType<Z3D::ZPointCloud::Ptr>("Z3D::PointCloud::Ptr");

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
