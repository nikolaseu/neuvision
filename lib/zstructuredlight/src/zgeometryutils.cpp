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

#include "zgeometryutils.h"

namespace Z3D
{

cv::Vec3d GeometryUtils::intersectLineWithLine3D(const cv::Vec3d &q1, const cv::Vec3d &v1, const cv::Vec3d &q2, const cv::Vec3d &v2, double *distance)
{
    cv::Vec3d q12 = q1 - q2;

    /// intermediate quantities
    double v1_dot_v1 = 1.f,//v1.dot(v1), /// assume we always use unit vectors for the direction
            v2_dot_v2 = 1.f,//v2.dot(v2), /// assume we always use unit vectors for the direction
            v1_dot_v2 = v1.dot(v2),
            q12_dot_v1 = q12.dot(v1),
            q12_dot_v2 = q12.dot(v2);

    /// calculate scale factors
    double s, t, denom;
    denom = v1_dot_v1*v2_dot_v2 - v1_dot_v2*v1_dot_v2;
    s =  (v1_dot_v2/denom)*q12_dot_v2 - (v2_dot_v2/denom)*q12_dot_v1;
    t = -(v1_dot_v2/denom)*q12_dot_v1 + (v1_dot_v1/denom)*q12_dot_v2;

    cv::Vec3d np1 = q1 + v1 * s;
    cv::Vec3d np2 = q2 + v2 * t;

    /// evaluate closest point
    cv::Vec3d p = 0.5f * (np1 + np2);

    if (distance != nullptr) {
        /// distance requested, we must calculate it
        cv::Vec3d diff = np1 - np2;
        *distance = diff.dot(diff);
        /// use squared distance, we only use it to compare between them
        ///*distance = cv::norm(diff);
    }

    return p;
}

} // namespace Z3D
