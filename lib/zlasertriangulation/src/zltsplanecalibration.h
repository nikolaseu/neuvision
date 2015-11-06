#ifndef Z3D_LASERTRIANGULATION___LTSPLANECALIBRATION_H
#define Z3D_LASERTRIANGULATION___LTSPLANECALIBRATION_H

#include "zlasertriangulation_global.h"
#include "zltscamera3dcalibrationinterface.h"

#include <Z3DCameraCalibration>

#include "opencv2/core/core.hpp"

namespace Z3D
{

class Z3D_LASERTRIANGULATION_SHARED_EXPORT LTSPlaneCalibration : public LTSCamera3DCalibrationInterface
{
public:
    LTSPlaneCalibration(Z3D::ZCameraCalibration::Ptr calibration);

    void getCalibratedPointForPixel(int x, int y, float *realX, float *realY, float *realZ);

    bool loadCalibration(const QString &fileName);

private:
    inline int indexForPixel(int x, int y) const { return x + y * m_sensorWidth; }

    void generateLookUpTable();

    /// camera calibration
    Z3D::ZCameraCalibration::Ptr m_cameraCalibration;

    /// sensor size (used to generate the look up table)
    int m_sensorWidth;
    int m_sensorHeight;

    /// distance to plane
    float m_planeDistance;

    /// plane normal vector
    float m_planeNormalX;
    float m_planeNormalY;
    float m_planeNormalZ;

    /// look up table
    std::vector<cv::Point3f> m_undistortedPoints;
};

} // namespace Z3D

#endif // Z3D_LASERTRIANGULATION___LTSPLANECALIBRATION_H
