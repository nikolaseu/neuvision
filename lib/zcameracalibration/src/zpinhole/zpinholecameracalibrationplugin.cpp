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

#include "ZCameraCalibration/zpinholecameracalibrationplugin.h"

#include "ZCameraCalibration/zopencvcustomstereomulticameracalibrator.h"
#include "ZCameraCalibration/zopencvstereocameracalibration.h"
#include "ZCameraCalibration/zopencvstereomulticameracalibrator.h"
#include "ZCameraCalibration/zopencvstereomulticameracalibratorconfigwidget.h"
#include "ZCameraCalibration/zpinholecameracalibrator.h"
#include "ZCameraCalibration/zpinholecameracalibratorconfigwidget.h"

#include "ZCameraCalibration/zpinholecameracalibration.h"

#include <opencv2/core/persistence.hpp>

#include <QDebug>
#include <QGroupBox>
#include <QLabel>
#include <QVBoxLayout>

namespace Z3D
{

ZMultiCameraCalibrationPtr readStereoCalibrationFile(QString fileName)
{
    cv::FileStorage fs(qPrintable(fileName), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        qWarning() << "cannot read stereo calibration file" << fileName;
        return nullptr;
    }

    cv::Mat cameraMatrix[2];
    cv::Mat distCoeffs[2];
    cv::Size imageSize[2];
    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;

    fs["leftCameraMatrix"] >> cameraMatrix[0];
    fs["leftDistortionCoeffs"] >> distCoeffs[0];
    std::vector<int> imageSizeVec;
    fs["leftImageSize"] >> imageSizeVec;
    imageSize[0] = cv::Size(imageSizeVec[0], imageSizeVec[1]);

    fs["rightCameraMatrix"] >> cameraMatrix[1];
    fs["rightDistortionCoeffs"] >> distCoeffs[1];
    imageSizeVec.clear();
    fs["rightImageSize"] >> imageSizeVec;
    imageSize[1] = cv::Size(imageSizeVec[0], imageSizeVec[1]);

    fs["R"] >> R;
    fs["T"] >> T;
    fs["E"] >> E;
    fs["F"] >> F;

    fs.release();

    return ZMultiCameraCalibrationPtr(
                new ZOpenCVStereoCameraCalibration(cameraMatrix, distCoeffs, imageSize, R, T, E, F));
}

ZCameraCalibrationPtr ZPinholeCameraCalibrationPlugin::getCalibration(QVariantMap options)
{
    if (!options.contains("ConfigFile")) {
        return nullptr;
    }

    QString configFileName = options.value("ConfigFile").toString();

    auto calibration = new ZPinholeCameraCalibration();
    calibration->loadFromFile(configFileName);

    return ZCameraCalibrationPtr(calibration);
}

QList<ZCameraCalibrator *> ZPinholeCameraCalibrationPlugin::getCameraCalibrators()
{
    return {
        new ZPinholeCameraCalibrator()
    };
}

QWidget *ZPinholeCameraCalibrationPlugin::getConfigWidget(ZCameraCalibrator *cameraCalibrator)
{
    const auto item = m_cameraCalibratorWidgets.find(cameraCalibrator);
    if (item != m_cameraCalibratorWidgets.end()) {
        return item->second;
    }

    QWidget *widget = nullptr;
    if (auto *pinholeCalibrator = qobject_cast<ZPinholeCameraCalibrator*>(cameraCalibrator)) {
        widget = new ZPinholeCameraCalibratorConfigWidget(pinholeCalibrator);
    }

    if (widget) {
        QObject::connect(cameraCalibrator, &ZCameraCalibrator::destroyed, [=](QObject *) {
            m_cameraCalibratorWidgets.erase(cameraCalibrator);
        });
        m_cameraCalibratorWidgets[cameraCalibrator] = widget;
    }

    return widget;
}

ZMultiCameraCalibrationPtr ZPinholeCameraCalibrationPlugin::getMultiCameraCalibration(QVariantMap options)
{
    if (!options.contains("ConfigFile")) {
        return nullptr;
    }

    QString configFileName = options.value("ConfigFile").toString();
    return readStereoCalibrationFile(configFileName);
}

bool ZPinholeCameraCalibrationPlugin::saveCalibration(const QString &fileName, ZMultiCameraCalibrationPtr calibration)
{
    auto stereoCalibration = std::dynamic_pointer_cast<ZOpenCVStereoCameraCalibration>(calibration);
    if (!stereoCalibration) {
        return false;
    }

    QString fileNameYML = fileName;
    if (!fileName.endsWith(".yml")) {
        fileNameYML += ".yml";
    }

    cv::FileStorage fs(qPrintable(fileNameYML), cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        qWarning() << "cannot save stereo calibration: cannot open file" << fileNameYML;
        return false;
    }

    fs << "leftCameraMatrix" << stereoCalibration->cameraMatrix[0];
    fs << "leftDistortionCoeffs" << stereoCalibration->distCoeffs[0];
    fs << "leftImageSize" << stereoCalibration->imageSize[0];

    fs << "rightCameraMatrix" << stereoCalibration->cameraMatrix[1];
    fs << "rightDistortionCoeffs" << stereoCalibration->distCoeffs[1];
    fs << "rightImageSize" << stereoCalibration->imageSize[1];

    fs << "R" << stereoCalibration->R;
    fs << "T" << stereoCalibration->T;
    fs << "E" << stereoCalibration->E;
    fs << "F" << stereoCalibration->F;

    fs.release();

    return true;
}

QList<ZMultiCameraCalibrator *> ZPinholeCameraCalibrationPlugin::getMultiCameraCalibrators()
{
    return {
        new ZOpenCVStereoMultiCameraCalibrator(),
        new ZOpenCVCustomStereoMultiCameraCalibrator()
    };
}

QWidget *ZPinholeCameraCalibrationPlugin::getConfigWidget(ZMultiCameraCalibrator *multiCameraCalibrator)
{
    const auto item = m_multiCameraCalibratorWidgets.find(multiCameraCalibrator);
    if (item != m_multiCameraCalibratorWidgets.end()) {
        return item->second;
    }

    QWidget *widget = nullptr;
    if (auto *opencvStereoCalibration = qobject_cast<ZOpenCVStereoMultiCameraCalibrator*>(multiCameraCalibrator)) {
        widget = new ZOpenCVStereoMultiCameraCalibratorConfigWidget(opencvStereoCalibration);
    } else if (/*auto *customStereoCalibration =*/ qobject_cast<ZOpenCVCustomStereoMultiCameraCalibrator*>(multiCameraCalibrator)) {
        widget = new QGroupBox(tr("Note"));
        QVBoxLayout *layout = new QVBoxLayout(widget);
        QLabel *label = new QLabel(
                    tr("<p><strong>This assumes the cameras are already calibrated!</strong></p>"
                       "<p>Only the rotation and translation between cameras will be calculated.</p>"),
                    widget);
        label->setWordWrap(true);
        layout->addWidget(label);
        widget->setLayout(layout);
    }

    if (widget) {
        QObject::connect(multiCameraCalibrator, &QObject::destroyed, [=](QObject *) {
            m_multiCameraCalibratorWidgets.erase(multiCameraCalibrator);
        });
        m_multiCameraCalibratorWidgets[multiCameraCalibrator] = widget;
    }

    return widget;
}

} // namespace Z3D
