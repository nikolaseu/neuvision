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

#include "zpinholecameracalibrationplugin.h"

#include "zpinholecameracalibration.h"

#include "zpinholecameracalibrator.h"
#include "zopencvcustomstereomulticameracalibrator.h"
#include "zopencvstereomulticameracalibrator.h"
#include "zpinholecameracalibratorconfigwidget.h"
#include "zopencvstereomulticameracalibratorconfigwidget.h"

#include <QGroupBox>
#include <QLabel>
#include <QVBoxLayout>

namespace Z3D
{

ZCameraCalibrationPtr ZPinholeCameraCalibrationPlugin::getCalibration(QVariantMap options)
{
    ZCameraCalibrationPtr calibration(new ZPinholeCameraCalibration);

    if (calibration && options.contains("ConfigFile")) {
        QString configFileName = options.value("ConfigFile").toString();
        calibration->loadFromFile(configFileName);
    }

    return calibration;
}

QList<ZCameraCalibrator *> ZPinholeCameraCalibrationPlugin::getCameraCalibrators()
{
    QList<ZCameraCalibrator *> list;

    list << new ZPinholeCameraCalibrator();

    return list;
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

QList<ZMultiCameraCalibrator *> ZPinholeCameraCalibrationPlugin::getMultiCameraCalibrators()
{
    QList<ZMultiCameraCalibrator *> list;

    list << new ZOpenCVStereoMultiCameraCalibrator();
    list << new ZOpenCVCustomStereoMultiCameraCalibrator();

    return list;
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
