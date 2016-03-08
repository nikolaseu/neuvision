#pragma once

#include "qwt_raster_data.h"

namespace Z3D
{

class ZCameraCalibration;

class ZCalibrationDistortionData : public QwtRasterData
{
public:
    ZCalibrationDistortionData(ZCameraCalibration *calibration);

    virtual double value( double x, double y ) const;

    void updateMaxDistortion();

//protected:
    ZCameraCalibration *m_calibration;
    double m_maxDistortion;
};

} // namespace Z3D
