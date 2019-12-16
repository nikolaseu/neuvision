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

#include "zstereoslsplugin.h"

#include "zdualcamerastereosls.h"
#include "zsinglecamerastereosls.h"

#include "ZCameraAcquisition/zcameraprovider.h"
#include "ZCameraCalibration/zcameracalibrationprovider.h"
#include "ZStructuredLight/zpatternprojectionprovider.h"

#include <QDebug>
#include <QLabel>
#include <QSettings>

namespace Z3D {

ZStereoSLSPlugin::ZStereoSLSPlugin()
{

}

ZStructuredLightSystemPtr ZStereoSLSPlugin::get(QSettings *settings)
{
    const QString mode = settings->value("Mode").toString();

    settings->beginGroup("StereoCalibration");
    auto stereoCalibration = ZCameraCalibrationProvider::getMultiCameraCalibration(settings);
    settings->endGroup();

    if (!stereoCalibration) {
        qWarning() << "failed to load stereo calibration for" << mode;
        return nullptr;
    }

    auto patternProjection = ZPatternProjectionProvider::get(settings);
    if (!patternProjection) {
        qWarning() << "failed to load pattern projection for" << mode;
        return nullptr;
    }

    if (mode == "DualCamera") {
        ZCameraList cameras;
        settings->beginGroup("Cameras");
            settings->beginGroup("Left");
                cameras.push_back(ZCameraProvider::getCamera(settings));
            settings->endGroup();

            settings->beginGroup("Right");
                cameras.push_back(ZCameraProvider::getCamera(settings));
            settings->endGroup();
        settings->endGroup();

        for (auto camera : cameras) {
            if (!camera) {
                qWarning() << "failed to load cameras for" << mode;
                return nullptr;
            }
        }

        return ZStructuredLightSystemPtr(new ZDualCameraStereoSLS(cameras, stereoCalibration, patternProjection));
    } else if (mode == "Projector+Camera") {
        settings->beginGroup("Camera");
        auto camera = ZCameraProvider::getCamera(settings);
        settings->endGroup();

        if (!camera) {
            qWarning() << "failed to load camera for" << mode;
            return nullptr;
        }

        return ZStructuredLightSystemPtr(new ZSingleCameraStereoSLS(camera, stereoCalibration, patternProjection));
    } else {
        qWarning() << "unknown ZStereoSLS mode:" << mode;
        return nullptr;
    }
}

} // namespace Z3D
