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
#include "ZCore/zlogging.h"
#include "ZStructuredLight/zpatternprojectionprovider.h"

#include <QLabel>
#include <QSettings>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zstructuredlight.slsystems.stereo", QtInfoMsg)

namespace Z3D
{

ZStereoSLSPlugin::ZStereoSLSPlugin()
{

}

ZStructuredLightSystemPtr ZStereoSLSPlugin::get(QSettings *settings)
{
    //! TODO replace QMap with std::unordered_map
    QMap<QString, std::function<ZStructuredLightSystemPtr(QSettings *, ZMultiCameraCalibrationPtr, ZPatternProjectionPtr)>> slsModes;

    slsModes.insert(
            "DualCamera",
            [](QSettings *settings,
               ZMultiCameraCalibrationPtr stereoCalibration,
               ZPatternProjectionPtr patternProjection) -> ZStructuredLightSystemPtr
            {
                ZCameraList cameras;
                settings->beginGroup("Cameras");
                settings->beginGroup("Left");
                cameras.push_back(ZCameraProvider::getCamera(settings));
                settings->endGroup();

                settings->beginGroup("Right");
                cameras.push_back(ZCameraProvider::getCamera(settings));
                settings->endGroup();
                settings->endGroup();

                for (const auto &camera : cameras) {
                    if (!camera) {
                        zWarning() << "failed to load cameras for DualCamera mode";
                        return nullptr;
                    }
                }

                return ZStructuredLightSystemPtr(
                        new ZDualCameraStereoSLS(cameras, stereoCalibration, patternProjection));
            });

    slsModes.insert(
            "Projector+Camera",
            [](QSettings *settings,
               ZMultiCameraCalibrationPtr stereoCalibration,
               ZPatternProjectionPtr patternProjection) -> ZStructuredLightSystemPtr
            {
                settings->beginGroup("Camera");
                auto camera = ZCameraProvider::getCamera(settings);
                settings->endGroup();

                if (!camera) {
                    zWarning() << "failed to load camera for Projector+Camera mode";
                    return nullptr;
                }

                return ZStructuredLightSystemPtr(
                        new ZSingleCameraStereoSLS(camera, stereoCalibration, patternProjection));
            });

    const QString mode = settings->value("Mode").toString();
    if (slsModes.find(mode) == slsModes.end()) {
        QStringList modesList;
        for (const auto &key : slsModes.keys()) {
            modesList << key;
        }
        zWarning() << "mode not found:" << mode
                   << "(available modes:" << modesList.join(", ") << ")";
        settings->setValue("Mode", QString("<FIXME chose one of: %1>").arg(modesList.join(", ")));

        return nullptr;
    }

    settings->beginGroup("StereoCalibration");
    auto stereoCalibration = ZCameraCalibrationProvider::getMultiCameraCalibration(settings);
    settings->endGroup();

    if (!stereoCalibration) {
        zWarning() << "failed to load stereo calibration for mode:" << mode;
        return nullptr;
    }

    auto patternProjection = ZPatternProjectionProvider::get(settings);
    if (!patternProjection) {
        zWarning() << "failed to load pattern projection for mode:" << mode;
        return nullptr;
    }

    return slsModes[mode](settings, stereoCalibration, patternProjection);
}

} // namespace Z3D
