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

#include "zqtgeometryloadersplugin.h"

#include "objgeometryloader.h"
#include "plygeometryloader.h"
#include "stlgeometryloader.h"

#include "zpointcloud.h"
#include "zpointfield.h"

#include <QDebug>
#include <QFile>
#include <QLoggingCategory>
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/private/qaxisalignedboundingbox_p.h>

using namespace Qt3DRender;

namespace Z3D
{

namespace // anonymous namespace
{
Q_LOGGING_CATEGORY(loggingCategory, "z3d.zpointcloud.plugins.zqtgeometryloaders")

class ZQtPointCloud : public ZPointCloud
{
public:
    explicit ZQtPointCloud(Qt3DRender::BaseGeometryLoader &loader)
    {
        const auto &m_points = loader.vertices();
        count = m_points.size();
        elementSize = 3 + (loader.hasNormals() ? 3 : 0);
        stride = elementSize * sizeof(float);
        bufferBytes.resize(stride * count);
        float *fptr = reinterpret_cast<float*>(bufferBytes.data());

        const bool hasNormals = loader.hasNormals();
        const auto &m_normals = loader.normals();
        for (int index = 0; index < count; ++index) {
            *fptr++ = m_points.at(index).x();
            *fptr++ = m_points.at(index).y();
            *fptr++ = m_points.at(index).z();

            if (hasNormals) {
                *fptr++ = m_normals.at(index).x();
                *fptr++ = m_normals.at(index).y();
                *fptr++ = m_normals.at(index).z();
            }
        } // of buffer filling loop

        // basic fiels always
        m_fields = {
            new ZPointField("x", 0, ZPointField::FLOAT32, 1),
            new ZPointField("y", 4, ZPointField::FLOAT32, 1),
            new ZPointField("z", 8, ZPointField::FLOAT32, 1),
        };

        // normals might not be available
        if (hasNormals) {
            m_fields.push_back(new ZPointField("normal_x", 12, ZPointField::FLOAT32, 1));
            m_fields.push_back(new ZPointField("normal_y", 16, ZPointField::FLOAT32, 1));
            m_fields.push_back(new ZPointField("normal_z", 20, ZPointField::FLOAT32, 1));
        }

        bb = QAxisAlignedBoundingBox(m_points);
    }

    void updateAttributes() override { }
    unsigned int height() const override { return 1; }
    unsigned int width() const override { return count; }
    unsigned int pointStep() const override { return stride; }
    unsigned int rowStep() const override { return width(); }

    QByteArray data() const override { return bufferBytes; }
    const std::vector<ZPointField *> &fields() const override { return m_fields; }
    QVector3D minimum() const override { return bb.minPoint(); }
    QVector3D maximum() const override { return bb.maxPoint(); }
    QVector3D center() const override { return bb.center(); }

private:
    int count;
    quint32 elementSize;
    quint32 stride;
    std::vector<ZPointField *> m_fields;
    QByteArray bufferBytes;
    QAxisAlignedBoundingBox bb;
};
}

ZQtGeometryLoadersPlugin::ZQtGeometryLoadersPlugin(QObject *parent)
    : QObject(parent)
{

}

ZPointCloudPtr ZQtGeometryLoadersPlugin::loadPointCloud(const QString &filename) const
{
    qDebug(loggingCategory) << "Trying to read PointCloud from" << filename;

    QFile file(filename);
    if (!file.open(QFile::ReadOnly)) {
        qWarning(loggingCategory) << "Failed to open file:" << filename;
        return nullptr;
    }

    std::shared_ptr<Qt3DRender::BaseGeometryLoader> loader;
    if (filename.endsWith(".ply", Qt::CaseInsensitive)) {
        loader.reset(new Qt3DRender::PlyGeometryLoader);
    }
    else if (filename.endsWith(".stl", Qt::CaseInsensitive)) {
        loader.reset(new Qt3DRender::StlGeometryLoader);
    }
    else if (filename.endsWith(".obj", Qt::CaseInsensitive)) {
        loader.reset(new Qt3DRender::ObjGeometryLoader);
    }

    if (loader && loader->load(&file)) {
        qDebug(loggingCategory) << "data loaded from" << filename << ", creating point cloud...";
        return ZPointCloudPtr(new ZQtPointCloud(*loader));
    }

    qWarning(loggingCategory) << "Failed to read PointCloud. Invalid/unrecognized format:" << filename;
    return nullptr;
}

} // namespace Z3D
