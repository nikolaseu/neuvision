#pragma once

#include "zpointcloud_global.h"
#include "zpointcloud_typedefs.h"

#include "pcl/PolygonMesh.h"
#include "pcl/search/kdtree.h"
#include "pcl/visualization/point_cloud_handlers.h"

#include <QSharedPointer>

namespace Z3D
{

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZPointCloud
{
public:
    typedef QSharedPointer<Z3D::ZPointCloud> Ptr;

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
