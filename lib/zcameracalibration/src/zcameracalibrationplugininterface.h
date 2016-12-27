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

#include "zcameracalibration_global.h"
#include "zcameracalibration.h"
#include "zcameracalibrator.h"
#include "zmulticameracalibrator.h"

#include "zcoreplugin.h"

namespace Z3D
{

class Z3D_CAMERACALIBRATION_SHARED_EXPORT ZCameraCalibrationPluginInterface : public ZCorePlugin
{
public:
    virtual ~ZCameraCalibrationPluginInterface() {}

    /// calibration utilities
    virtual ZCameraCalibration::Ptr getCalibration(QVariantMap options) = 0;

    virtual QList<ZCameraCalibrator *> getCameraCalibrators() = 0;

    virtual QList<ZMultiCameraCalibrator *> getMultiCameraCalibrators() = 0;
};

} // namespace Z3D

#define ZCameraCalibrationPluginInterface_iid "z3d.zcameracalibration.zcameracalibrationplugininterface"

Q_DECLARE_INTERFACE(Z3D::ZCameraCalibrationPluginInterface, ZCameraCalibrationPluginInterface_iid)
