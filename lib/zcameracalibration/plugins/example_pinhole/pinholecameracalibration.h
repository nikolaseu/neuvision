#ifndef Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATION_H
#define Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATION_H

#include "cameracalibrationinterface.h"
#include "opencv2/core/core.hpp"

namespace Z3D
{

class PinholeCameraCalibration : public CameraCalibrationInterface
{
public:
    PinholeCameraCalibration();

    virtual void getCalibratedPixel(int x, int y,
                                    float *calibratedX, float *calibratedY);

    virtual void getRayForPixel(int x, int y, /// pixel coordinates
                                float *origin, /// ray origin
                                float *direction); /// ray direction

    virtual bool loadCalibration(const QString &fileName);

protected:
    inline int indexForPixel(int x, int y) const { return x + y * m_sensorWidth; }

    void generateLookUpTable();

    cv::Matx33f m_cvRotation;
    cv::Vec3f m_cvTranslation;

    cv::Mat m_cvCameraMatrix;
    cv::Mat m_cvDistortionCoeffs;

    std::vector<cv::Point2f> m_undistortedPoints;
    //std::vector<cv::Vec3f> m_undistortedRays;
    //std::vector<cv::Vec3f> m_undistortedWorldRays;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATION___PINHOLECAMERACALIBRATION_H
