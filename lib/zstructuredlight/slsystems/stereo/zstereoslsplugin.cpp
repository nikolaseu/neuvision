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
#include "zdualcamerastereoslsconfigwidget.h"
#include "zsinglecamerastereosls.h"

#include <QLabel>

namespace Z3D {

ZStereoSLSPlugin::ZStereoSLSPlugin()
{

}

QString ZStereoSLSPlugin::id() const
{
    return "ZStereoSLS";
}

QString ZStereoSLSPlugin::name() const
{
    return "Stereo";
}

QString ZStereoSLSPlugin::version() const
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

QWidget *ZStereoSLSPlugin::getConfigWidget(ZStructuredLightSystem *structuredLightSystem)
{
    if (auto *dualCameraStereo = qobject_cast<ZDualCameraStereoSLS *>(structuredLightSystem)) {
        static QWidget *widget = nullptr;
        if (!widget) {
            widget = new ZDualCameraStereoSLSConfigWidget(dualCameraStereo);
        }

        return widget;
    }

    if (/*auto *singleCameraStereo =*/ qobject_cast<ZSingleCameraStereoSLS *>(structuredLightSystem)) {
        static QWidget *widget = nullptr;
        if (!widget) {
            widget = new QLabel("Not implemented yet :(");
        }

        return widget;
    }

    return nullptr;
}

} // namespace Z3D
