#ifndef ZPOINTCLOUDDATA_H
#define ZPOINTCLOUDDATA_H

#include "zsimplepointcloud.h"

#include <qopengl.h>
#include <QVector>

class ZPointCloudData
{
public:
    typedef QSharedPointer<ZPointCloudData> Ptr;

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
