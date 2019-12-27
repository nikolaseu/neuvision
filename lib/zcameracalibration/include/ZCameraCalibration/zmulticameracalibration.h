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

#include "zcameracalibration_fwd.h"
#include "zcameracalibration_global.h"

namespace Z3D
{

class Z3D_CAMERACALIBRATION_SHARED_EXPORT ZMultiCameraCalibration
{
public:
    explicit ZMultiCameraCalibration(const std::vector<ZCameraCalibrationPtr> &calibrations);

    virtual ~ZMultiCameraCalibration();

    std::vector<ZCameraCalibrationPtr> calibrations() const;

private:
    const std::vector<ZCameraCalibrationPtr> m_calibrations;
};

} // namespace Z3D
