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

#ifndef Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATIONPLUGIN_H
#define Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATIONPLUGIN_H

#include <QObject>
#include <QVariantMap>

#include "cameracalibrationplugininterface.h"

namespace Z3D
{

class PinholeCameraCalibrationPlugin : public QObject, CameraCalibrationPluginInterface
{
    Q_OBJECT

#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "z3d.cameracalibration.cameracalibrationplugininterface" FILE "pinhole.json")
#endif // QT_VERSION >= 0x050000

    Q_INTERFACES(Z3D::CameraCalibrationPluginInterface)

public:
    /// plugin information
    virtual QString name();
    virtual QString version();

    /// calibration utilities
    virtual CameraCalibrationInterface::Ptr getCalibration(QVariantMap options);
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATIONPLUGIN_H
