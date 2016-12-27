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

#include <QSharedPointer>
#include <QString>

namespace Z3D
{

class LTSCamera3DCalibrationInterface
{

public:
    typedef QSharedPointer<Z3D::LTSCamera3DCalibrationInterface> Ptr;

    virtual ~LTSCamera3DCalibrationInterface() {}

    virtual void getCalibratedPointForPixel(int x, int y, float *calibratedX, float *calibratedY, float *calibratedZ) = 0;

    virtual bool loadCalibration(const QString &fileName) = 0;
};

} // namespace Z3D
