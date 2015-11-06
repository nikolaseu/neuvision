#include "geometryutils.h"

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
