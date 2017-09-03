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

#include "zincompletecirclegridpatternfinderplugin.h"

#include "zincompletecirclegridpatternfinder.h"
#include "zincompletecirclegridpatternfinderconfigwidget.h"

namespace Z3D {

QString ZIncompleteCircleGridPatternFinderPlugin::id() const
{
    return QString(metaObject()->className());
}

QString ZIncompleteCircleGridPatternFinderPlugin::name() const
{
    return QString(metaObject()->className());
}

QString ZIncompleteCircleGridPatternFinderPlugin::version() const
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCalibrationPatternFinder::Ptr> ZIncompleteCircleGridPatternFinderPlugin::getPatternFinders()
{
    return QList<ZCalibrationPatternFinder::Ptr>() << ZCalibrationPatternFinder::Ptr(new ZIncompleteCircleGridPatternFinder());
}

QWidget *ZIncompleteCircleGridPatternFinderPlugin::getConfigWidget(ZCalibrationPatternFinder *patternFinder)
{
    const auto item = m_patternFinderWidgets.find(patternFinder);
    if (item != m_patternFinderWidgets.end()) {
        return item->second;
    }

    QWidget *widget = nullptr;
    if (auto *finder = qobject_cast<ZIncompleteCircleGridPatternFinder *>(patternFinder)) {
        widget = new ZIncompleteCircleGridPatternFinderConfigWidget(finder);
    }

    if (widget) {
        QObject::connect(patternFinder, &ZCalibrationPatternFinder::destroyed, [=](QObject *) {
            m_patternFinderWidgets.erase(patternFinder);
        });
        m_patternFinderWidgets[patternFinder] = widget;
    }

    return widget;
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zincompletecirclegridcalibrationpatternfinderplugin, Z3D::ZIncompleteCircleGridPatternFinderPlugin)
#endif
