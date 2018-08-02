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

#include "zpointcloudgeometry.h"

#include "zpointcloud.h"
#include "zpointfield.h"

#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QBuffer>

namespace Z3D
{

QByteArray createColorBufferFromBGRA(ZPointCloud *pointCloud, uint colorOffset)
{
    struct BGRA // just to simplify converting color data
    {
        uint8_t b;
        uint8_t g;
        uint8_t r;
        uint8_t a;
    };

    const uint colorChannels = 4; // ABGR
    const uint size = pointCloud->width() * pointCloud->height();
    const uint pointStep = pointCloud->pointStep();
    QByteArray data = pointCloud->data();
    QByteArray colorData;
    colorData.resize(int(sizeof(uint8_t)*size*colorChannels));
    int32_t *colorDataFloats = reinterpret_cast<int32_t*>(colorData.data());
    for (uint i=colorOffset; i<size ; i++) {
        BGRA bgra;
        memcpy(&bgra, &(data.data()[colorOffset + i * pointStep]), sizeof(BGRA));
        colorDataFloats[i] = int32_t(bgra.a) << 24
                           | int32_t(bgra.b) << 16
                           | int32_t(bgra.g) << 8
                           | int32_t(bgra.r);
    }
    return colorData;
}

class QPointCloudGeometryPrivate
{
public:
    Qt3DRender::QBuffer *m_vertexBuffer = nullptr;
    Qt3DRender::QBuffer *m_colorBuffer = nullptr;
    ZPointCloud *m_pointCloud = nullptr;
    uint m_colorOffset = 0;
    bool m_colorAvailable = false;
};

ZPointCloudGeometry::ZPointCloudGeometry(Qt3DCore::QNode *parent)
    : Qt3DRender::QGeometry(parent)
    , m_p(new QPointCloudGeometryPrivate)
{
    m_p->m_vertexBuffer = new Qt3DRender::QBuffer(this);
    m_p->m_colorBuffer = new Qt3DRender::QBuffer(this);
}

ZPointCloudGeometry::~ZPointCloudGeometry()
{
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
}

void ZPointCloudGeometry::updateVertices()
{
    if (m_p->m_pointCloud->data().length() == 0) {
        return;
    }

    updateAttributes();

    m_p->m_vertexBuffer->setData(m_p->m_pointCloud->data());
    if (m_p->m_colorAvailable) {
        m_p->m_colorBuffer->setData(createColorBufferFromBGRA(m_p->m_pointCloud, m_p->m_colorOffset));
    }
}

ZPointCloud *ZPointCloudGeometry::pointCloud() const
{
    return m_p->m_pointCloud;
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
            qDebug() << "skipped index";
        }
    }

    // Prepare hash table to query attribute names easily
    QHash<QString, ZPointField* > pfs;
    m_p->m_pointCloud->updateAttributes();

    const std::vector<ZPointField *> &fieldList = m_p->m_pointCloud->fields();
    for (auto field : fieldList) {
        qDebug() << "found field:" << field->name();
        pfs.insert(field->name(), field);
    }

    // parse point fields and make reasonable attributes out of them
    auto pf = pfs.find("x");
    if (pf != pfs.cend()) {
        qDebug() << "Adding position attribute from individual x y z w ...";
        const uint num = 1 + (pfs.contains("y")?1:0) + (pfs.contains("z")?1:0) + (pfs.contains("w")?1:0);
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(m_p->m_vertexBuffer,
                                                                    Qt3DRender::QAttribute::defaultPositionAttributeName(),
                                                                    pointFieldTypeToAttributeType((*pf)->dataType()),
                                                                    num,
                                                                    m_p->m_pointCloud->width() * m_p->m_pointCloud->height(),
                                                                    (*pf)->offset(),
                                                                    m_p->m_pointCloud->pointStep(),
                                                                    this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);
        setBoundingVolumePositionAttribute(attrib);
    }

    pf = pfs.find("rgb");
    if (pf != pfs.cend()) {
        qDebug() << "Adding color attribute from rgb ...";
        const uint num = 4; // RGBA == 4 bytes
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(m_p->m_colorBuffer,
                                                                    Qt3DRender::QAttribute::defaultColorAttributeName(),
                                                                    Qt3DRender::QAttribute::UnsignedByte,
                                                                    num,
                                                                    m_p->m_pointCloud->width() * m_p->m_pointCloud->height(),
                                                                    0,
                                                                    sizeof(uint32_t),
                                                                    this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);

        m_p->m_colorOffset = (*pf)->offset();
        m_p->m_colorAvailable = true;
    }

    pf = pfs.find("normal_x");
    if (pf != pfs.cend()) {
        qDebug() << "Adding normal/curvature attribute ...";
        const uint num = 1 + (pfs.contains("normal_y")?1:0) + (pfs.contains("normal_z")?1:0) + (pfs.contains("curvature")?1:0);
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(m_p->m_vertexBuffer,
                                                                    Qt3DRender::QAttribute::defaultNormalAttributeName(),
                                                                    pointFieldTypeToAttributeType((*pf)->dataType()),
                                                                    num,
                                                                    m_p->m_pointCloud->width() * m_p->m_pointCloud->height(),
                                                                    (*pf)->offset(),
                                                                    m_p->m_pointCloud->pointStep(),
                                                                    this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);
    }

    pf = pfs.find("intensity");
    if (pf != pfs.cend()) {
        qDebug() << "Adding intensity attribute ...";
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(m_p->m_vertexBuffer,
                                                                    "intensity",
                                                                    pointFieldTypeToAttributeType((*pf)->dataType()),
                                                                    1,
                                                                    m_p->m_pointCloud->width() * m_p->m_pointCloud->height(),
                                                                    (*pf)->offset(),
                                                                    m_p->m_pointCloud->pointStep(),
                                                                    this);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        addAttribute(attrib);
    }
}

void ZPointCloudGeometry::setPointCloud(ZPointCloud *pointCloud)
{
    if (m_p->m_pointCloud == pointCloud) {
        return;
    }

    m_p->m_pointCloud = pointCloud;
    updateVertices();

    emit pointCloudChanged(pointCloud);
}

} // namespace Z3D
