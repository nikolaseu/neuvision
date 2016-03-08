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
