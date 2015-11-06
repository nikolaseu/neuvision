#include "zopencvvideocaptureplugin.h"
#include "zopencvvideocapturecamera.h"

#include <QDebug>

namespace Z3D
{

ZOpenCVVideoCapturePlugin::ZOpenCVVideoCapturePlugin()
{

}

ZOpenCVVideoCapturePlugin::~ZOpenCVVideoCapturePlugin()
{

}

QString ZOpenCVVideoCapturePlugin::id()
{
    return QString("ZOpenCV");
}

QString ZOpenCVVideoCapturePlugin::name()
{
    return QString("OpenCV VideoCapture");
}

QString ZOpenCVVideoCapturePlugin::version()
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCameraInfo *> ZOpenCVVideoCapturePlugin::getConnectedCameras()
{
    QList<ZCameraInfo*> camerasList;

    int cameraDeviceID = 0;
    while (true) {
        cv::VideoCapture *capture = new cv::VideoCapture(cameraDeviceID);
        if (capture->isOpened()) {
            delete capture;
            capture = 0;

            QVariantMap extraData;
            extraData["DeviceID"] = cameraDeviceID;

            camerasList << new ZCameraInfo(this, QString("Camera %1").arg(cameraDeviceID), extraData);
        } else {
            delete capture;
            capture = 0;

            /// if capture device didn't open, assume there is no more
            break;
        }

        cameraDeviceID++;
    }

    return camerasList;
}

ZCameraInterface::Ptr ZOpenCVVideoCapturePlugin::getCamera(QVariantMap options)
{
    int cameraDeviceID = options.value("DeviceID").toInt();

    OpenCVVideoCaptureCamera *camera = 0;

    cv::VideoCapture *capture = new cv::VideoCapture(cameraDeviceID);
    if (capture->isOpened()) {
        delete capture;
        capture = 0;
        camera = new OpenCVVideoCaptureCamera(cameraDeviceID);
    } else {
        delete capture;
        capture = 0;
    }

    if (camera) {
        return ZCameraInterface::Ptr(camera);
    }

    qDebug() << "opencv camera not found:" << options;

    return ZCameraInterface::Ptr(0);
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zopencvvideocapture, Z3D::ZOpenCVVideoCapturePlugin)
#endif
