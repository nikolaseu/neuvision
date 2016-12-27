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

#include "pinholecameracalibrationplugin.h"

#include "pinholecameracalibration.h"

namespace Z3D
{

QString PinholeCameraCalibrationPlugin::name()
{
    return QString("Pinhole");
}

QString PinholeCameraCalibrationPlugin::version()
{
    return QString("1.0");
}

CameraCalibrationInterface::Ptr PinholeCameraCalibrationPlugin::getCalibration(QVariantMap options)
{
    CameraCalibrationInterface::Ptr calibration(new PinholeCameraCalibration);

    if (calibration && options.contains("ConfigFile")) {
        QString configFileName = options.value("ConfigFile").toString();
        calibration->loadCalibration(configFileName);
    }

    return calibration;
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zpinholeplugin, Z3D::PinholeCameraCalibrationPlugin)
#endif // QT_VERSION < 0x050000

