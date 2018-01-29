/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "zcameracalibrationplugininterface.h"

#include <QObject>
#include <QVariantMap>

namespace Z3D
{

class ZPinholeCameraCalibrationPlugin : public QObject, public ZCameraCalibrationPluginInterface
{
    Q_OBJECT

    // ZCameraCalibrationPluginInterface interface
public:
    ZCameraCalibrationPtr getCalibration(QVariantMap options) override;
    QList<ZCameraCalibrator *> getCameraCalibrators() override;
    QWidget *getConfigWidget(ZCameraCalibrator *cameraCalibrator) override;

    ZMultiCameraCalibrationPtr getMultiCameraCalibration(QVariantMap options) override;
    bool saveCalibration(const QString &fileName, ZMultiCameraCalibrationPtr calibration) override;

    QList<ZMultiCameraCalibrator *> getMultiCameraCalibrators() override;
    QWidget *getConfigWidget(ZMultiCameraCalibrator *multiCameraCalibrator) override;

private:
    std::map<ZCameraCalibrator *, QWidget *> m_cameraCalibratorWidgets;
    std::map<ZMultiCameraCalibrator *, QWidget *> m_multiCameraCalibratorWidgets;
};

} // namespace Z3D
