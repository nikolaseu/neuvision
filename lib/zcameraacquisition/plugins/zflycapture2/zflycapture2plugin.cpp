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

#include "zflycapture2plugin.h"

#include "zflycapture2camera.h"

namespace Z3D
{

QString ZFlyCapture2Plugin::id() const
{
    return QString("ZFlyCapture2");
}

QString ZFlyCapture2Plugin::displayName() const
{
    return QString("PGR FlyCapture2 SDK");
}

QString ZFlyCapture2Plugin::version() const
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCameraInfo *> ZFlyCapture2Plugin::getConnectedCameras()
{
    //! TODO
    QList<ZCameraInfo *> list;

    return list;
}

ZCameraPtr ZFlyCapture2Plugin::getCamera(QVariantMap options)
{
    QList<ZCameraPtr> cameraList = ZFlyCapture2Camera::getConnectedCameras();
    if (cameraList.size())
        return cameraList.first();

    return nullptr;
}

} // namespace Z3D
