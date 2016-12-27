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

namespace Z3D
{

QString ZPinholeCameraCalibrationPlugin::id()
{
    return QString(PINHOLE_CALIB_NAME);
}

QString ZPinholeCameraCalibrationPlugin::name()
{
    return QString(PINHOLE_CALIB_NAME);
}

QString ZPinholeCameraCalibrationPlugin::version()
{
    return QString(PINHOLE_CALIB_VERSION);
}

ZCameraCalibration::Ptr ZPinholeCameraCalibrationPlugin::getCalibration(QVariantMap options)
{
    ZCameraCalibration::Ptr calibration(new ZPinholeCameraCalibration);

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

QList<ZMultiCameraCalibrator *> ZPinholeCameraCalibrationPlugin::getMultiCameraCalibrators()
{
    QList<ZMultiCameraCalibrator *> list;

    list << new ZOpenCVCustomStereoMultiCameraCalibrator();
    list << new ZOpenCVStereoMultiCameraCalibrator();

    return list;
}

} // namespace Z3D

/*
#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zpinholeplugin, Z3D::PinholeCameraCalibrationPlugin)
#endif // QT_VERSION < 0x050000
*/
