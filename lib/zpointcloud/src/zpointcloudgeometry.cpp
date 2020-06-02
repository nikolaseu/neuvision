//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2018 Nicolas Ulrich <nikolaseu@gmail.com>
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

#include "ZPointCloud/zpointcloudgeometry.h"

#include "ZPointCloud/zpointcloud.h"
#include "ZPointCloud/zpointfield.h"

#include <ZCore/zlogging.h>

#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QBuffer>
#include <cmath>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zpointcloud", QtInfoMsg)

namespace Z3D
{

class ZPointCloudGeometryPrivate
{
public:
    ZPointCloud *pointCloud = nullptr;

    Qt3DRender::QBuffer *vertexBuffer = nullptr;
    Qt3DRender::QBuffer *indexBuffer = nullptr;

    uint levelOfDetail = 0; // 0 == original data
};

ZPointCloudGeometry::ZPointCloudGeometry(Qt3DCore::QNode *parent)
    : Qt3DRender::QGeometry(parent)
    , m_p(new ZPointCloudGeometryPrivate)
{
    zDebug() << "creating" << this;
}

ZPointCloudGeometry::~ZPointCloudGeometry()
{
    zDebug() << "destroying" << this;
    delete m_p;
}

Qt3DRender::QAttribute::VertexBaseType pointFieldTypeToAttributeType(const ZPointField::PointFieldTypes &inp)
{
    switch (inp) {
    case ZPointField::INT8:
        return Qt3DRender::QAttribute::Byte;
    case ZPointField::INT16:
        return Qt3DRender::QAttribute::Short;
    case ZPointField::INT32:
        return Qt3DRender::QAttribute::Int;
    case ZPointField::UINT8:
        return Qt3DRender::QAttribute::UnsignedByte;
    case ZPointField::UINT16:
        return Qt3DRender::QAttribute::UnsignedShort;
    case ZPointField::UINT32:
        return Qt3DRender::QAttribute::UnsignedInt;
    case ZPointField::FLOAT32:
        return Qt3DRender::QAttribute::Float;
    case ZPointField::FLOAT64:
        return Qt3DRender::QAttribute::Double;
    }

    zCritical() << "unhandled point field type" << inp;
    return Qt3DRender::QAttribute::Byte;
}

ZPointCloud *ZPointCloudGeometry::pointCloud() const
{
    return m_p->pointCloud;
}

uint ZPointCloudGeometry::levelOfDetail() const
{
    return m_p->levelOfDetail;
}

void ZPointCloudGeometry::updateAttributes()
{
    const std::vector<ZPointField*> &fieldList = m_p->pointCloud->fields();
    auto findField = [&fieldList](const QString &name) -> ZPointField* {
        const auto it = std::find_if(fieldList.cbegin(), fieldList.cend(), [&name](const auto &field){
            return field->name() == name;
        });
        return it != fieldList.cend()
                   ? *it
                   : nullptr;
    };

    // parse point fields and make reasonable attributes out of them
    if (const auto *pf = findField("position")) {
        zDebug() << "Adding position attribute, field:" << pf;
        auto *attrib = new Qt3DRender::QAttribute(m_p->vertexBuffer,
                                                  Qt3DRender::QAttribute::defaultPositionAttributeName(),
                                                  pointFieldTypeToAttributeType(pf->dataType()),
                                                  pf->count(),
                                                  m_p->pointCloud->vertexCount(),
                                                  pf->offset(),
                                                  m_p->pointCloud->pointStep(),
                                                  this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);
        setBoundingVolumePositionAttribute(attrib);
    }

    if (const auto *pf = findField("rgb")) {
        zDebug() << "Adding color attribute, field:" << pf;
        auto *attrib = new Qt3DRender::QAttribute(m_p->vertexBuffer,
                                                  Qt3DRender::QAttribute::defaultColorAttributeName(),
                                                  pointFieldTypeToAttributeType(pf->dataType()),
                                                  pf->count(),
                                                  m_p->pointCloud->vertexCount(),
                                                  pf->offset(),
                                                  m_p->pointCloud->pointStep(),
                                                  this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);
    }

    if (const auto *pf = findField("normal")) {
        zDebug() << "Adding normal attribute, field:" << pf;
        auto *attrib = new Qt3DRender::QAttribute(m_p->vertexBuffer,
                                                  Qt3DRender::QAttribute::defaultNormalAttributeName(),
                                                  pointFieldTypeToAttributeType(pf->dataType()),
                                                  pf->count(),
                                                  m_p->pointCloud->vertexCount(),
                                                  pf->offset(),
                                                  m_p->pointCloud->pointStep(),
                                                  this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);
    }

    if (const auto *pf = findField("radii")) {
        zDebug() << "Adding radii attribute, field:" << pf;
        auto *attrib = new Qt3DRender::QAttribute(m_p->vertexBuffer,
                                                  "vertexRadii",
                                                  pointFieldTypeToAttributeType(pf->dataType()),
                                                  pf->count(),
                                                  m_p->pointCloud->vertexCount(),
                                                  pf->offset(),
                                                  m_p->pointCloud->pointStep(),
                                                  this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);
    }

    /// If what we have is actually a mesh, also include faces (indices)
    if (m_p->pointCloud->hasTriangles()) {
        // use UINT - no conversion needed, but let's ensure int is 32-bit!
        Q_ASSERT(sizeof(int) == sizeof(quint32));
        auto *indexAttribute = new Qt3DRender::QAttribute(m_p->indexBuffer,
                                                          Qt3DRender::QAttribute::UnsignedInt,
                                                          1,
                                                          m_p->pointCloud->trianglesCount() * 3);
        indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
        addAttribute(indexAttribute);
    }
}

void ZPointCloudGeometry::setPointCloud(ZPointCloud *pointCloud)
{
    if (m_p->pointCloud == pointCloud) {
        return;
    }

    /// remove all attributes
    const QVector<Qt3DRender::QAttribute *> atts = attributes();
    for (Qt3DRender::QAttribute *attr : atts) {
        removeAttribute(attr);
        attr->deleteLater();
    }

    /// update point cloud
    m_p->pointCloud = pointCloud;

    /// recreate buffers, changing data to previous buffer can be dangerous if old
    /// data is not available anymore...
    if (m_p->vertexBuffer) {
        m_p->vertexBuffer->deleteLater();
    }
    m_p->vertexBuffer = new Qt3DRender::QBuffer(this);
    m_p->vertexBuffer->setUsage(Qt3DRender::QBuffer::StaticDraw);
    m_p->vertexBuffer->setData(m_p->pointCloud->vertexData());

    if (m_p->indexBuffer) {
        m_p->indexBuffer->deleteLater();
    }
    m_p->indexBuffer = new Qt3DRender::QBuffer(this);
    m_p->indexBuffer->setUsage(Qt3DRender::QBuffer::StaticDraw);
    m_p->indexBuffer->setData(m_p->pointCloud->trianglesData());

    /// add new attributes
    updateAttributes();

    emit pointCloudChanged(m_p->pointCloud);
}

void ZPointCloudGeometry::setLevelOfDetail(uint levelOfDetail)
{
    /*if (m_p->levelOfDetail == levelOfDetail) {
        return;
    }

    //! FIXME this is mostly a quick workaround, we should do something better ...

    const int levelOfDetailDiff = int(levelOfDetail) - int(m_p->levelOfDetail);
    float levelOfDetailChangeFactor = 1.f;
    if (levelOfDetailDiff > 0) {
        // we should lower the level of detail
        levelOfDetailChangeFactor = 1.f / powf(2.f, levelOfDetailDiff);
    }
    else {
        // we should increase the level of detail
        levelOfDetailChangeFactor = powf(2.f, -levelOfDetailDiff);
    }

    zDebug(loggingCategory) << "changing level of detail from" << m_p->levelOfDetail << "to" << levelOfDetail << "factor" << levelOfDetailChangeFactor;
    for (auto &attr : attributes()) {
        attr->setCount(uint(levelOfDetailChangeFactor * attr->count()));
        attr->setByteStride(uint(attr->byteStride() / levelOfDetailChangeFactor));
    }

    m_p->levelOfDetail = levelOfDetail;

    emit levelOfDetailChanged(m_p->levelOfDetail);*/
}

} // namespace Z3D
