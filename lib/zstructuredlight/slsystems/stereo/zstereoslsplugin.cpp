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

namespace Z3D {

ZStereoSLSPlugin::ZStereoSLSPlugin()
{

}

QString ZStereoSLSPlugin::id()
{
    return "ZStereoSLS";
}

QString ZStereoSLSPlugin::name()
{
    return "Stereo";
}

QString ZStereoSLSPlugin::version()
{
    return Z3D_VERSION_STR;
}

QList<QString> ZStereoSLSPlugin::getAll()
{
    return QList<QString>()
            << "DualCamera"
            << "Projector+Camera";
}

ZStructuredLightSystem::Ptr ZStereoSLSPlugin::get(QSettings *settings)
{
    ZStructuredLightSystem::Ptr sls;

    const QString mode = settings->value("Mode").toString();
    if (mode == "DualCamera") {
        sls = ZStructuredLightSystem::Ptr(new ZDualCameraStereoSLS());
    } else if (mode == "Projector+Camera") {
        sls = ZStructuredLightSystem::Ptr(new ZSingleCameraStereoSLS());
    }

    if (sls) {
        sls->init(settings);
    }

    return sls;
}

} // namespace Z3D
