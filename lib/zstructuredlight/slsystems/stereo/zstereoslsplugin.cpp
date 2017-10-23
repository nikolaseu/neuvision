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
    } else {
        qWarning() << "unknown ZStereoSLS mode:" << mode;
    }

    if (sls) {
        sls->init(settings);
    }

    return sls;
}

QWidget *ZStereoSLSPlugin::getConfigWidget(ZStructuredLightSystem *structuredLightSystem)
{
    const auto item = m_structuredLightSystemWidgets.find(structuredLightSystem);
    if (item != m_structuredLightSystemWidgets.end()) {
        return item->second;
    }

    QWidget *widget = nullptr;
    if (auto *dualCameraStereo = qobject_cast<ZDualCameraStereoSLS *>(structuredLightSystem)) {
        widget = new ZDualCameraStereoSLSConfigWidget(dualCameraStereo);
    } else if (/*auto *singleCameraStereo =*/ qobject_cast<ZSingleCameraStereoSLS *>(structuredLightSystem)) {
        widget = new QLabel("Not implemented yet :(");
    }

    if (widget) {
        QObject::connect(structuredLightSystem, &QObject::destroyed, [=](QObject *) {
            m_structuredLightSystemWidgets.erase(structuredLightSystem);
        });
        m_structuredLightSystemWidgets[structuredLightSystem] = widget;
    }

    return widget;
}

} // namespace Z3D
