#include "ZPointCloud/zpointcloud_io.h"

#include "ZPointCloud/zpointcloud.h"

#include <ZCore/zlogging.h>

#include <QDataStream>
#include <QFile>
#include <QTextStream>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zpointcloud", QtInfoMsg)

namespace Z3D::ZPointCloudIO
{

bool savePLY(const Z3D::ZPointCloud &cloud, const QString &fileName)
{
    //! FIXME this method should handle correctly the data available,
    //! we should not assume x y z nx ny nz rgba and triangles.
    //! That's why we have point fields!

    if (!fileName.endsWith(".ply", Qt::CaseInsensitive)) {
        zWarning() << "file extension should be *.ply, it is:" << fileName;
        return false;
    }

    QFile file(fileName);
    if (!file.open(QFile::WriteOnly)) {
        zWarning() << "cannot open file for writing:" << fileName;
        return false;
    }

    QTextStream headerStream(&file);
    headerStream << "ply\n"
                 << "format binary_little_endian 1.0\n"
                 << "comment Created with Z3D\n"
                 << "element vertex " << cloud.vertexCount() << "\n"
                 << "property float x\n"
                 << "property float y\n"
                 << "property float z\n";

    if (cloud.hasNormals()) {
        headerStream << "property float nx\n"
                     << "property float ny\n"
                     << "property float nz\n";
    }

    if (cloud.hasColors()) {
        headerStream << "property uchar red\n"
                     << "property uchar green\n"
                     << "property uchar blue\n"
                     << "property uchar alpha\n";
    }

    if (cloud.hasRadii()) {
        headerStream << "property float radii\n";
    }

    const auto faceCount = cloud.trianglesCount();
    if (cloud.hasTriangles()) {
        headerStream << "element face " << faceCount << "\n"
                     << "property list uchar int vertex_indices\n";
    }

    headerStream << "end_header\n";
    headerStream.flush();

    QDataStream binaryDataStream(&file);
    binaryDataStream.writeRawData(cloud.vertexData().constData(), cloud.vertexData().size());

    if (cloud.hasTriangles()) {
        const uint32_t *indices = reinterpret_cast<const uint32_t *>(cloud.trianglesData().constData());
        for (size_t faceIndex = 0; faceIndex < faceCount; ++faceIndex) {
            // we only have triangles
            constexpr uchar three[1]{3};
            binaryDataStream.writeRawData(reinterpret_cast<const char *>(three), 1);

            const uint32_t faceData[3]{indices[faceIndex * 3 + 0],
                                       indices[faceIndex * 3 + 1],
                                       indices[faceIndex * 3 + 2]};
            binaryDataStream.writeRawData(reinterpret_cast<const char *>(faceData), 3 * 4);
        }
    }

    file.close();

    return true;
}

} // Z3D::ZPointCloudIO
