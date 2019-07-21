#include "zqtgeometryloaders.h"

#include "objgeometryloader.h"
#include "plygeometryloader.h"
#include "stlgeometryloader.h"

#include "zpointcloud.h"
#include "zpointfield.h"

#include <QDebug>
#include <QFile>
#include <QLoggingCategory>
#include <Qt3DRender/QBuffer>
//#include <Qt3DRender/private/qaxisalignedboundingbox_p.h>

using namespace Qt3DRender;

namespace Z3D
{

namespace ZQtGeometryLoaders
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
        elementSize = 3
            + (loader.hasNormals() ? 3 : 0)
            + (loader.hasVertexColors() ? 1 : 0);
        stride = elementSize * sizeof(float);
        bufferBytes.resize(stride * count);

        qDebug(loggingCategory) << "creating point cloud with" << count << "points, size:" << bufferBytes.size() << "bytes";

        float *fptr = reinterpret_cast<float*>(bufferBytes.data());

        const bool hasNormals = loader.hasNormals();
        const bool hasColors = loader.hasVertexColors();
        const auto &m_normals = loader.normals();
        const auto &m_colors = loader.vertexColors();
        for (int index = 0; index < count; ++index) {
            *fptr++ = m_points.at(index).x();
            *fptr++ = m_points.at(index).y();
            *fptr++ = m_points.at(index).z();

            if (hasNormals) {
                *fptr++ = m_normals.at(index).x();
                *fptr++ = m_normals.at(index).y();
                *fptr++ = m_normals.at(index).z();
            }

            if (hasColors) {
                *fptr++ = m_colors.at(index);
            }
        } // of buffer filling loop

        // basic fiels always
        m_fields = {
            new ZPointField("x", 0, ZPointField::FLOAT32, 1),
            new ZPointField("y", 4, ZPointField::FLOAT32, 1),
            new ZPointField("z", 8, ZPointField::FLOAT32, 1),
        };

        unsigned int offset = 12;

        // normals might not be available
        if (hasNormals) {
            m_fields.push_back(new ZPointField("normal_x", offset+0, ZPointField::FLOAT32, 1));
            m_fields.push_back(new ZPointField("normal_y", offset+4, ZPointField::FLOAT32, 1));
            m_fields.push_back(new ZPointField("normal_z", offset+8, ZPointField::FLOAT32, 1));
            offset += 12;
        }

        if (hasColors) {
            m_fields.push_back(new ZPointField("rgb", offset+0, ZPointField::FLOAT32, 1));
            offset += 4;
        }
    }

    void updateAttributes() override { }
    unsigned int height() const override { return 1; }
    unsigned int width() const override { return count; }
    unsigned int pointStep() const override { return stride; }
    unsigned int rowStep() const override { return width(); }

    QByteArray data() const override { return bufferBytes; }
    const std::vector<ZPointField *> &fields() const override { return m_fields; }

private:
    int count = 0;
    quint32 elementSize = 0;
    quint32 stride = 0;
    std::vector<ZPointField *> m_fields;
    QByteArray bufferBytes;
};

}

ZPointCloudPtr loadPointCloud(const QString &fileName)
{
    qDebug(loggingCategory) << "Trying to read PointCloud from" << fileName;

    QFile file(fileName);
    if (!file.open(QFile::ReadOnly)) {
        qWarning(loggingCategory) << "Failed to open file:" << fileName;
        return nullptr;
    }

    std::shared_ptr<Qt3DRender::BaseGeometryLoader> loader;
    if (fileName.endsWith(".ply", Qt::CaseInsensitive)) {
        loader.reset(new Qt3DRender::PlyGeometryLoader);
    }
    else if (fileName.endsWith(".stl", Qt::CaseInsensitive)) {
        loader.reset(new Qt3DRender::StlGeometryLoader);
    }
    else if (fileName.endsWith(".obj", Qt::CaseInsensitive)) {
        loader.reset(new Qt3DRender::ObjGeometryLoader);
    }

    if (loader && loader->load(&file)) {
        qDebug(loggingCategory) << "data loaded from" << fileName << ", creating point cloud...";
        return ZPointCloudPtr(new ZQtPointCloud(*loader));
    }

    qWarning(loggingCategory) << "Failed to read PointCloud. Invalid/unrecognized format:" << fileName;
    return nullptr;
}

} // namespace ZQtGeometryLoaders

} // namespace Z3D
