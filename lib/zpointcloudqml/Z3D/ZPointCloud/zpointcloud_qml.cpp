#include "zpointcloud_qml.h"

#include "ZPointCloud/zpointcloud_io.h"

#include <QtCore/QUrl>

namespace Z3D::ZPointCloudInternal
{

ZPointCloud_QML::ZPointCloud_QML(QObject *parent)
    : QObject(parent)
{

}

bool ZPointCloud_QML::savePLY(Z3D::ZPointCloud *cloud, const QUrl &fileUrl)
{
    return Z3D::ZPointCloudIO::savePLY(*cloud, fileUrl.toLocalFile());
}

} // namespace Z3D::ZPointCloudInternal
