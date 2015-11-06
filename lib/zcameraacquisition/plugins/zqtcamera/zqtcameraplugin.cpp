#include "zqtcameraplugin.h"
#include "zqtcamera.h"

#include <QCameraInfo>

namespace Z3D {

QString ZQtCameraPlugin::id()
{
    return QStringLiteral("ZQTCAMERA");
}

QString ZQtCameraPlugin::name()
{
    return tr("QtMultimedia camera");
}

QString ZQtCameraPlugin::version()
{
    return QStringLiteral(Z3D_VERSION_STR);
}

QList<ZCameraInfo *> ZQtCameraPlugin::getConnectedCameras()
{
    QList<ZCameraInfo *> cameraList;

    QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
    qDebug() << id() << "found" << cameras.size() << "cameras.";
    foreach (const QCameraInfo &qtcameraInfo, cameras) {
        QVariantMap extraData;
        extraData["description"] = qtcameraInfo.description();
        extraData["deviceName"] = qtcameraInfo.deviceName();
        cameraList << new ZCameraInfo(this, qtcameraInfo.description(), extraData);
    }

    return cameraList;
}

ZCameraInterface::Ptr ZQtCameraPlugin::getCamera(QVariantMap options)
{
    QByteArray deviceName = options["deviceName"].toByteArray();

    if (deviceName.isNull() || deviceName.isEmpty())
        return ZCameraInterface::Ptr(0);

    QCamera *qcamera = new QCamera(deviceName);

    if (qcamera->isAvailable())
        return ZCameraInterface::Ptr(new ZQtCamera(qcamera));

    delete qcamera;
    return ZCameraInterface::Ptr(0);
}

} // namespace Z3D


#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zqtcamera, Z3D::ZQtCameraPlugin)
#endif // QT_VERSION < 0x050000
