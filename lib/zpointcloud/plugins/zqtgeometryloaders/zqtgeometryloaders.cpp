#include "zqtgeometryloaders.h"

#include "objgeometryloader.h"
#include "plygeometryloader.h"
#include "stlgeometryloader.h"

#include <ZCore/zlogging.h>
#include <ZPointCloud/zpointcloud.h>
#include <ZPointCloud/zpointfield.h>

#include <QtCore/QFile>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zpointcloud.zqtgeometryloaders", QtDebugMsg)

namespace Z3D::ZQtGeometryLoaders
{

using namespace Qt3DRender;

namespace // anonymous namespace
{

class ZQtPointCloud Q_DECL_FINAL : public ZPointCloud
{
public:
    static ZPointCloudUniquePtr create(Qt3DRender::BaseGeometryLoader &loader)
    {
        const auto &points = loader.vertices();
        const auto count = points.size();
        const auto elementSize = 3
                      + (loader.hasNormals() ? 3 : 0)
                      + (loader.hasVertexColors() ? 1 : 0);
        const auto stride = elementSize * sizeof(float);

        QByteArray bufferBytes;
        bufferBytes.resize(stride * count);

        zDebug() << "creating point cloud with" << count << "points, size:" << bufferBytes.size() << "bytes";

        float *fptr = reinterpret_cast<float*>(bufferBytes.data());

        const bool hasNormals = loader.hasNormals();
        const bool hasColors = loader.hasVertexColors();
        const auto &m_normals = loader.normals();
        const auto &m_colors = loader.vertexColors();
        for (int index = 0; index < count; ++index) {
            *fptr++ = points.at(index).x();
            *fptr++ = points.at(index).y();
            *fptr++ = points.at(index).z();

            if (hasNormals) {
                *fptr++ = m_normals.at(index).x();
                *fptr++ = m_normals.at(index).y();
                *fptr++ = m_normals.at(index).z();
            }

            if (hasColors) {
                *fptr++ = m_colors.at(index);
            }
        } // of buffer filling loop

        std::vector<ZPointField *> fields {
            // position always available
            ZPointField::position(0, ZPointField::FLOAT32, 3)
        };

        unsigned int offset = 12;

        // normals might not be available
        if (hasNormals) {
            fields.push_back(ZPointField::normal(offset, ZPointField::FLOAT32, 3));
            offset += 12;
        }

        if (hasColors) {
            fields.push_back(ZPointField::color(offset, ZPointField::UINT8, 4));
            offset += 4;
        }

        return ZPointCloudUniquePtr(new ZQtPointCloud(1, // height
                                                      count, // width
                                                      stride, // point step
                                                      count * stride, // row step
                                                      fields,
                                                      bufferBytes,
                                                      loader.indices()));
    }

    [[nodiscard]] QByteArray vertexData() const override { return m_bufferBytes; }

    [[nodiscard]] QByteArray trianglesData() const override
    {
        /// do not copy data, but we need to be careful!
        return QByteArray::fromRawData(reinterpret_cast<const char*>(m_indices.constData()),
                                       int(sizeof(quint32)) * m_indices.size());
    }

private:
    explicit ZQtPointCloud(unsigned int height,
                           unsigned int width,
                           unsigned int pointStep,
                           unsigned int rowStep,
                           const std::vector<ZPointField *> &fields,
                           const QByteArray &bufferBytes,
                           const QVector<unsigned int> &indices,
                           QObject *parent = nullptr)
        : ZPointCloud(height, width, pointStep, rowStep, fields, parent)
        , m_bufferBytes(bufferBytes)
        , m_indices(indices)
    {

    }

    QByteArray m_bufferBytes;
    QVector<unsigned int> m_indices;
};

} // anonymous namespace

ZPointCloudUniquePtr loadPointCloud(const QString &fileName)
{
    zDebug() << "Trying to read PointCloud from" << fileName;

    QFile file(fileName);
    if (!file.open(QFile::ReadOnly)) {
        zWarning() << "Failed to open file:" << fileName;
        return nullptr;
    }

    std::shared_ptr<Qt3DRender::BaseGeometryLoader> loader;
    if (fileName.endsWith(".ply", Qt::CaseInsensitive)) {
        loader = std::make_shared<Qt3DRender::PlyGeometryLoader>();
    } else if (fileName.endsWith(".stl", Qt::CaseInsensitive)) {
        loader = std::make_shared<Qt3DRender::StlGeometryLoader>();
    } else if (fileName.endsWith(".obj", Qt::CaseInsensitive)) {
        loader = std::make_shared<Qt3DRender::ObjGeometryLoader>();
    }

    if (loader && loader->load(&file)) {
        zDebug().nospace() << "data loaded from " << fileName << ", creating point cloud...";
        return ZQtPointCloud::create(*loader);
    }

    zWarning() << "Failed to read PointCloud. Invalid/unrecognized format:" << fileName;
    return nullptr;
}

} // namespace Z3D::ZQtGeometryLoaders
