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

#include <QLoggingCategory>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QBuffer>
#include <cmath>

namespace Z3D
{

namespace // anonymous namespace
{

Q_LOGGING_CATEGORY(loggingCategory, "z3d.zpointcloud.zpointcloudgeometry", QtInfoMsg)

} // anonymous namespace

class ZPointCloudGeometryPrivate
{
public:
    ZPointCloud *pointCloud = nullptr;

    Qt3DRender::QBuffer *vertexBuffer = nullptr;
    Qt3DRender::QBuffer *indexBuffer = nullptr;

    Qt3DRender::QAttribute* normalsAttrib = nullptr;
    Qt3DRender::QAttribute* colorsAttrib = nullptr;
    Qt3DRender::QAttribute *indexAttribute = nullptr;

    uint levelOfDetail = 0; // 0 == original data
};

ZPointCloudGeometry::ZPointCloudGeometry(Qt3DCore::QNode *parent)
    : Qt3DRender::QGeometry(parent)
    , m_p(new ZPointCloudGeometryPrivate)
{
    qDebug(loggingCategory) << "creating" << this;
    m_p->vertexBuffer = new Qt3DRender::QBuffer(this);
    m_p->indexBuffer = new Qt3DRender::QBuffer(this);
}

ZPointCloudGeometry::~ZPointCloudGeometry()
{
    qDebug(loggingCategory) << "destroying" << this;
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

    qCritical(loggingCategory) << "unhandled point field type" << inp;
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

bool ZPointCloudGeometry::hasNormals() const
{
    return m_p->normalsAttrib != nullptr;
}

bool ZPointCloudGeometry::hasColors() const
{
    return m_p->colorsAttrib != nullptr;
}

bool ZPointCloudGeometry::hasTriangles() const
{
    return m_p->indexAttribute != nullptr;
}

void ZPointCloudGeometry::updateAttributes()
{
    // completely rebuild attribute list and remove all previous attributes
    QVector<Qt3DRender::QAttribute *> atts = attributes();
    for (Qt3DRender::QAttribute *attr : atts) {
        if (attr->attributeType() == Qt3DRender::QAttribute::VertexAttribute) {
            removeAttribute(attr);
            attr->deleteLater();
        }
        else {
            qDebug(loggingCategory) << "skipped index";
        }
    }

    // Prepare hash table to query attribute names easily
    QHash<QString, ZPointField* > pfs;
    m_p->pointCloud->updateAttributes();

    const std::vector<ZPointField*> &fieldList = m_p->pointCloud->fields();
    for (auto field : fieldList) {
        qDebug(loggingCategory) << "found field" << field;
        pfs.insert(field->name(), field);
    }

    // parse point fields and make reasonable attributes out of them
    auto pf = pfs.find("x");
    if (pf != pfs.cend()) {
        qDebug(loggingCategory) << "Adding position attribute from individual x y z [w]";
        const uint num = 1 + (pfs.contains("y")?1:0) + (pfs.contains("z")?1:0) + (pfs.contains("w")?1:0);
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(m_p->vertexBuffer,
                                                                    Qt3DRender::QAttribute::defaultPositionAttributeName(),
                                                                    pointFieldTypeToAttributeType((*pf)->dataType()),
                                                                    num,
                                                                    m_p->pointCloud->width() * m_p->pointCloud->height(),
                                                                    (*pf)->offset(),
                                                                    m_p->pointCloud->pointStep(),
                                                                    this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);
        setBoundingVolumePositionAttribute(attrib);
    }

    pf = pfs.find("rgb");
    if (pf != pfs.cend()) {
        qDebug(loggingCategory) << "Adding color attribute from rgb";
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(m_p->vertexBuffer,
                                                                    Qt3DRender::QAttribute::defaultColorAttributeName(),
                                                                    pointFieldTypeToAttributeType((*pf)->dataType()),
                                                                    (*pf)->count(),
                                                                    m_p->pointCloud->width() * m_p->pointCloud->height(),
                                                                    (*pf)->offset(),
                                                                    m_p->pointCloud->pointStep(),
                                                                    this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);

        m_p->colorsAttrib = attrib;
    }
    else {
        m_p->colorsAttrib = nullptr;
    }

    pf = pfs.find("normal_x");
    if (pf != pfs.cend()) {
        qDebug(loggingCategory) << "Adding normal/curvature attribute";
        const uint num = 1 + (pfs.contains("normal_y")?1:0) + (pfs.contains("normal_z")?1:0) + (pfs.contains("curvature")?1:0);
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(m_p->vertexBuffer,
                                                                    Qt3DRender::QAttribute::defaultNormalAttributeName(),
                                                                    pointFieldTypeToAttributeType((*pf)->dataType()),
                                                                    num,
                                                                    m_p->pointCloud->width() * m_p->pointCloud->height(),
                                                                    (*pf)->offset(),
                                                                    m_p->pointCloud->pointStep(),
                                                                    this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);

        m_p->normalsAttrib = attrib;
    }
    else {
        m_p->normalsAttrib = nullptr;
    }

    pf = pfs.find("intensity");
    if (pf != pfs.cend()) {
        qDebug(loggingCategory) << "Adding intensity attribute, field:" << (*pf);
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(m_p->vertexBuffer,
                                                                    "intensity",
                                                                    pointFieldTypeToAttributeType((*pf)->dataType()),
                                                                    1,
                                                                    m_p->pointCloud->width() * m_p->pointCloud->height(),
                                                                    (*pf)->offset(),
                                                                    m_p->pointCloud->pointStep(),
                                                                    this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);
    }

    /// If what we have is actually a mesh, also include faces (indices)
    //! FIXME
    if (m_p->pointCloud->indices().size() > 0) {
        // use UINT - no conversion needed, but let's ensure int is 32-bit!
        Q_ASSERT(sizeof(int) == sizeof(quint32));
        Qt3DRender::QAttribute *indexAttribute = new Qt3DRender::QAttribute(m_p->indexBuffer,
                                                                            Qt3DRender::QAttribute::UnsignedInt,
                                                                            1,
                                                                            m_p->pointCloud->indices().size());
        indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
        addAttribute(indexAttribute);

        m_p->indexAttribute = indexAttribute;
    } else {
        m_p->indexAttribute = nullptr;
    }
}

void ZPointCloudGeometry::setPointCloud(ZPointCloud *pointCloud)
{
    if (m_p->pointCloud == pointCloud) {
        return;
    }

    m_p->pointCloud = pointCloud;

    updateAttributes();

    m_p->vertexBuffer->setData(m_p->pointCloud->vertexData());

    //! TODO this is making a copy, it's supposed to be unnecessary but seems like
    //! original data was being deleted so it was crashing when using picking for example
    m_p->indexBuffer->setData(QByteArray(reinterpret_cast<const char*>(m_p->pointCloud->indices().data()),
                                                      int(sizeof(quint32)) * m_p->pointCloud->indices().size()));

    emit hasNormalsChanged(hasNormals());
    emit hasColorsChanged(hasColors());
    emit hasTrianglesChanged(hasTriangles());
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

    qDebug(loggingCategory) << "changing level of detail from" << m_p->levelOfDetail << "to" << levelOfDetail << "factor" << levelOfDetailChangeFactor;
    for (auto &attr : attributes()) {
        attr->setCount(uint(levelOfDetailChangeFactor * attr->count()));
        attr->setByteStride(uint(attr->byteStride() / levelOfDetailChangeFactor));
    }

    m_p->levelOfDetail = levelOfDetail;

    emit levelOfDetailChanged(m_p->levelOfDetail);*/
}

} // namespace Z3D
