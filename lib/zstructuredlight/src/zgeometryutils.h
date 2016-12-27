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

#include <opencv2/core/core.hpp>

namespace GeometryUtils
{

/**
 * @brief GeometryUtils::intersectLineWithLine3D
 * Finds the closest 3D point between two 3D lines defined in parametric form
 * (i.e., containing a point Q and spanned by the vector V).
 * This function does not check for parallel lines, and assumes the direction
 * vector is a unit vector.
 *
 * @param q1 point on line1
 * @param v1 direction of line1
 * @param q2 point on line2
 * @param v2 direction of line2
 * @param distance the squared minimum distance between the lines
 * @return approximate intersection between lines
 */
cv::Vec3d intersectLineWithLine3D(const cv::Vec3d &q1,
                                  const cv::Vec3d &v1,
                                  const cv::Vec3d &q2,
                                  const cv::Vec3d &v2,
                                  double *distance = nullptr);

} // namespace GeometryUtils
