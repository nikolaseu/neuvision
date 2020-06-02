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

#include "zringgridpatternfinderplugin.h"

#include "zringgridpatternfinder.h"
#include "zringgridpatternfinderconfigwidget.h"

namespace Z3D
{

QList<ZCalibrationPatternFinderPtr> ZRingGridPatternFinderPlugin::getPatternFinders()
{
    return QList<ZCalibrationPatternFinderPtr>() << ZCalibrationPatternFinderPtr(new ZRingGridPatternFinder());
}

QWidget *ZRingGridPatternFinderPlugin::getConfigWidget(ZCalibrationPatternFinder *patternFinder)
{
    const auto item = m_patternFinderWidgets.find(patternFinder);
    if (item != m_patternFinderWidgets.end()) {
        return item->second;
    }

    QWidget *widget = nullptr;
    if (auto *finder = qobject_cast<ZRingGridPatternFinder *>(patternFinder)) {
        widget = new ZRingGridPatternFinderConfigWidget(finder);
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
