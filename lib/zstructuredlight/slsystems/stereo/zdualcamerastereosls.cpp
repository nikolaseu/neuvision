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

#include "zdualcamerastereosls.h"

#include "ZCameraAcquisition/zcamerainterface.h"
#include "ZCameraAcquisition/zcamerapreviewer.h"
#include "ZCameraCalibrator/zmulticameracalibratorwidget.h"
#include "ZCore/zsettingsitem.h"
#include "ZPointCloud/zpointcloud.h"
#include "ZStructuredLight/zdecodedpattern.h"

#include <QDebug>
#include <QSettings>
#include <QElapsedTimer>

namespace Z3D
{

ZDualCameraStereoSLS::ZDualCameraStereoSLS(ZCameraList cameras,
                                           ZMultiCameraCalibrationPtr stereoCalibration,
                                           ZPatternProjectionPtr patternProjection,
                                           QObject *parent)
    : ZStereoSLS(cameras, stereoCalibration, patternProjection, parent)
    , m_cameras(cameras)
{
    if (m_cameras.size() != 2) {
        qWarning() << "there must be exactly 2 cameras!";
        return;
    }

    const QString calibrationSettings("Calibration");

    ZSettingsItemPtr openCalibrationOption = std::make_unique<ZSettingsItemCommand>(calibrationSettings, "Change calibration ...", "Opens structured light system calibration window",
                                                                                    [&]() -> bool {
                                                                                        Z3D::ZMultiCameraCalibratorWidget *calibWidget = new Z3D::ZMultiCameraCalibratorWidget(m_cameras);
                                                                                        calibWidget->show();
                                                                                        return true;
                                                                                    });

    const QString leftCameraSettings("Left camera");

    ZSettingsItemPtr leftCameraPreviewOption = std::make_unique<ZSettingsItemCommand>(leftCameraSettings, "Preview ...", "Opens camera preview window",
                                                                                      [&]() -> bool {
                                                                                          auto *previewDialog = new Z3D::ZCameraPreviewer(m_cameras[0]);
                                                                                          previewDialog->show();
                                                                                          return true;
                                                                                      });

    ZSettingsItemPtr leftCameraSettingsOption = std::make_unique<ZSettingsItemCommand>(leftCameraSettings, "Settings ...", "Opens camera settings window",
                                                                                       [&]() -> bool {
                                                                                           m_cameras[0]->showSettingsDialog();
                                                                                           return true;
                                                                                       });

    const QString rightCameraSettings("Right camera");

    ZSettingsItemPtr rightCameraPreviewOption = std::make_unique<ZSettingsItemCommand>(rightCameraSettings, "Preview ...", "Opens camera preview window",
                                                                                       [&]() -> bool {
                                                                                           auto *previewDialog = new Z3D::ZCameraPreviewer(m_cameras[1]);
                                                                                           previewDialog->show();
                                                                                           return true;
                                                                                       });

    ZSettingsItemPtr rightCameraSettingsOption = std::make_unique<ZSettingsItemCommand>(rightCameraSettings, "Settings ...", "Opens camera settings window",
                                                                                        [&]() -> bool {
                                                                                            m_cameras[1]->showSettingsDialog();
                                                                                            return true;
                                                                                        });

    const QString advancedSettings("Advanced options");

    ZSettingsItemPtr maxValidDistanceOption = std::make_unique<ZSettingsItemFloat>(advancedSettings, "Max. valid distance", "Maximum distance to be considered valid",
                                                                                   std::bind(&ZDualCameraStereoSLS::maxValidDistance, this),
                                                                                   std::bind(&ZDualCameraStereoSLS::setMaxValidDistance, this, std::placeholders::_1),
                                                                                   0.0, // minimum
                                                                                   1.0); // maximum
    QObject::connect(this, &ZDualCameraStereoSLS::maxValidDistanceChanged,
                     maxValidDistanceOption.get(), &ZSettingsItem::valueChanged);

    const QString debugOptions("Debug options");

    ZSettingsItemPtr showDecodedPatternOption = std::make_unique<ZSettingsItemBool>(debugOptions, "Show decoded patterns", "Display decoded patterns as images (in a new window)",
                                                                                    std::bind(&ZDualCameraStereoSLS::debugShowDecodedImages, this),
                                                                                    std::bind(&ZDualCameraStereoSLS::setDebugShowDecodedImages, this, std::placeholders::_1));
    QObject::connect(this, &ZDualCameraStereoSLS::debugShowDecodedImagesChanged,
                     showDecodedPatternOption.get(), &ZSettingsItem::valueChanged);

    m_settings = {
        openCalibrationOption,
        leftCameraPreviewOption,
        leftCameraSettingsOption,
        rightCameraPreviewOption,
        rightCameraSettingsOption,
        maxValidDistanceOption,
        showDecodedPatternOption
    };
}

ZDualCameraStereoSLS::~ZDualCameraStereoSLS()
{

}

const std::vector<ZSettingsItemPtr> &ZDualCameraStereoSLS::settings()
{
    qDebug() << "returning settings for" << this;
    return m_settings;
}

ZCameraList ZDualCameraStereoSLS::cameras() const
{
    return m_cameras;
}

void ZDualCameraStereoSLS::onPatternProjected(ZProjectedPatternPtr pattern)
{
    Q_UNUSED(pattern)
}

void ZDualCameraStereoSLS::onPatternsDecoded(std::vector<ZDecodedPatternPtr> decodedPatterns)
{
    QElapsedTimer startTime;
    startTime.start();

    Z3D::ZPointCloudPtr cloud = triangulate(decodedPatterns[0]->intensityImg(),
            decodedPatterns[0]->decodedImage(),
            decodedPatterns[1]->decodedImage());

    if (cloud) {
        qDebug() << "finished calculating point cloud with" << cloud->width() * cloud->height()
                 << "points in" << startTime.elapsed() << "msecs";
        emit scanFinished(cloud);
    } else {
        qDebug() << "finished calculating point cloud in" << startTime.elapsed() << "msecs";
    }
}

} // namespace Z3D
