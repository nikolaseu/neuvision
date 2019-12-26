#include "ZPointCloud/zpointcloud_io.h"

#include "ZPointCloud/zpointcloud.h"

#include <QDataStream>
#include <QFile>
#include <QLoggingCategory>
#include <QTextStream>

namespace Z3D::ZPointCloudIO
{

namespace
{
Q_LOGGING_CATEGORY(loggingCategory, "z3d.zpointcloud.io", QtInfoMsg);
}

bool savePLY(const Z3D::ZPointCloud &cloud, const QString &fileName)
{
    //! FIXME this method should handle correctly the data available,
    //! we should not assume x y z nx ny nz rgba and triangles.
    //! That's why we have point fields!

    if (!fileName.endsWith(".ply", Qt::CaseInsensitive)) {
        qWarning(loggingCategory) << "file extension should be *.ply, it is:" << fileName;
        return false;
    }

    QFile file(fileName);
    if (!file.open(QFile::WriteOnly)) {
        qWarning(loggingCategory) << "cannot open file for writing:" << fileName;
        return false;
    }

    const size_t vertexCount = cloud.width() * cloud.height();

    QTextStream headerStream(&file);
    headerStream << "ply\n"
                 << "format binary_little_endian 1.0\n"
                 << "comment Created with Z3D\n"
                 << "element vertex " << vertexCount << "\n"
                 << "property float x\n"
                 << "property float y\n"
                 << "property float z\n";

    //    if (cloud->hasNormals()) {
        headerStream << "property float nx\n"
                     << "property float ny\n"
                     << "property float nz\n";
    //    }

//    if (cloud->hasColors()) {
        headerStream << "property uchar red\n"
                     << "property uchar green\n"
                     << "property uchar blue\n"
                     << "property uchar alpha\n";
//    }

    const size_t faceCount = cloud.indices().size() / 3; // we only handle triangular meshes
//    if (cloud.hasTriangles()) {
        headerStream << "element face " << faceCount << "\n"
                     << "property list uchar int vertex_indices\n";
//    }

    headerStream << "end_header\n";
    headerStream.flush();

    QDataStream binaryDataStream(&file);
    binaryDataStream.writeRawData(cloud.vertexData().constData(), cloud.vertexData().size());

//    if (cloud.hasTriangles()) {
        const auto &indices = cloud.indices();
        for (size_t faceIndex = 0; faceIndex < faceCount; ++faceIndex) {
            // we only have triangles
            constexpr uchar three[1]{3};
            binaryDataStream.writeRawData(reinterpret_cast<const char *>(three), 1);

            const uint32_t faceData[3]{indices[faceIndex * 3 + 0],
                                       indices[faceIndex * 3 + 1],
                                       indices[faceIndex * 3 + 2]};
            binaryDataStream.writeRawData(reinterpret_cast<const char *>(faceData), 3 * 4);
        }
//    }

    file.close();

    return true;
}

} // Z3D::ZPointCloudIO
