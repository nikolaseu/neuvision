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

#include "zopencvvideocaptureplugin.h"

#include "zopencvvideocapturecamera.h"

#include "ZCameraAcquisition/zcamerainfo.h"
#include "ZCore/zlogging.h"

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcameraacquisition.zopencvvideocapture", QtInfoMsg)

namespace Z3D
{

ZOpenCVVideoCapturePlugin::ZOpenCVVideoCapturePlugin()
{

}

ZOpenCVVideoCapturePlugin::~ZOpenCVVideoCapturePlugin()
{

}

QString ZOpenCVVideoCapturePlugin::displayName() const
{
    return QString("OpenCV VideoCapture");
}

QList<ZCameraInfo *> ZOpenCVVideoCapturePlugin::getConnectedCameras()
{
    QList<ZCameraInfo*> camerasList;

    int cameraDeviceID = 0;
    while (true) {
        auto *capture = new cv::VideoCapture(cameraDeviceID);
        if (capture->isOpened()) {
            delete capture;
            capture = nullptr;

            QVariantMap extraData;
            extraData["DeviceID"] = cameraDeviceID;

            camerasList << new ZCameraInfo(this, QString("Camera %1").arg(cameraDeviceID), extraData);
        } else {
            delete capture;
            capture = nullptr;

            /// if capture device didn't open, assume there is no more
            break;
        }

        cameraDeviceID++;
    }

    /*
    /// for example starting raspberry camera with:
    /// while true; do raspivid -o - -t 0 --mode 4 -w 1640 -h 1232 -fps 10 -b 15000000 -n | nc -l 5000; done;
    QString streamUrl("tcp://192.168.0.15:5000");
    QVariantMap extraData;
    extraData["StreamingURL"] = streamUrl;
    camerasList << new ZCameraInfo(this, QString("Stream at %1").arg(streamUrl), extraData);

    QString streamUrl2("tcp://192.168.0.14:5000");
    QVariantMap extraData2;
    extraData2["StreamingURL"] = streamUrl2;
    camerasList << new ZCameraInfo(this, QString("Stream at %1").arg(streamUrl2), extraData2);
    */

    return camerasList;
}

ZCameraPtr ZOpenCVVideoCapturePlugin::getCamera(QVariantMap options)
{
    OpenCVVideoCaptureCamera *camera = nullptr;

    if (options.contains("StreamingURL")) {
        /// for example starting raspberry camera with:
        /// while true; do raspivid -o - -t 0 --mode 4 -w 1640 -h 1232 -fps 10 -b 15000000 -n | nc -l 5000; done;
        const QString cameraStreamingUrl = options.value("StreamingURL").toString();
        cv::VideoCapture *capture = new cv::VideoCapture(cameraStreamingUrl.toStdString());
        if (capture->isOpened()) {
            camera = new OpenCVVideoCaptureCamera(capture);
        } else {
            delete capture;
            capture = nullptr;
        }
    }
    else {
        int cameraDeviceID = options.value("DeviceID").toInt();
        auto *capture = new cv::VideoCapture(cameraDeviceID);
        if (capture->isOpened()) {
            camera = new OpenCVVideoCaptureCamera(capture);
        } else {
            delete capture;
            capture = nullptr;
        }
    }

    if (camera) {
        return ZCameraPtr(camera);
    }

    zDebug() << "opencv camera not found:" << options;

    return nullptr;
}

} // namespace Z3D
