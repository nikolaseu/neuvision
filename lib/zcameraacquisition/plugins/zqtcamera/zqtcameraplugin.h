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

#include "ZCameraAcquisition/zcameraplugininterface.h"

namespace Z3D {

class ZQtCameraPlugin : public QObject, public ZCameraPluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "z3d.cameraacquisition.cameraplugininterface" FILE "zqtcamera.json")
    Q_INTERFACES(Z3D::ZCameraPluginInterface)

public:
    /// plugin information
    QString displayName() const override;

    /// camera utilities
    QList<ZCameraInfo *> getConnectedCameras() override;
    ZCameraPtr getCamera(QVariantMap options) override;
};

} // namespace Z3D
