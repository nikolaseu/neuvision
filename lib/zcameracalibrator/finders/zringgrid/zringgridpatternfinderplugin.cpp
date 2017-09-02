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

namespace Z3D {

QString ZRingGridPatternFinderPlugin::id() const
{
    return QString(metaObject()->className());
}

QString ZRingGridPatternFinderPlugin::name() const
{
    return QString(metaObject()->className());
}

QString ZRingGridPatternFinderPlugin::version() const
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCalibrationPatternFinder::Ptr> ZRingGridPatternFinderPlugin::getPatternFinders()
{
    return QList<ZCalibrationPatternFinder::Ptr>() << ZCalibrationPatternFinder::Ptr(new ZRingGridPatternFinder());
}

QWidget *ZRingGridPatternFinderPlugin::getConfigWidget(ZCalibrationPatternFinder::Ptr patternFinder)
{
    if (auto *finder = qobject_cast<ZRingGridPatternFinder *>(patternFinder.data())) {
        /// TODO improve this, this assumes there will always be only one of each type
        static QWidget *widget = nullptr;
        if (!widget) {
            widget = new ZRingGridPatternFinderConfigWidget(finder);
            widget->setVisible(false);
        }

        return widget;
    }

    return nullptr;
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zringgridcalibrationpatternfinderplugin, Z3D::ZRingGridPatternFinderPlugin)
#endif
