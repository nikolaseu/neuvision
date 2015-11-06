#include "zcalibrationdistortionplotdata.h"

#include "zcameracalibration.h"

namespace Z3D
{

ZCalibrationDistortionData::ZCalibrationDistortionData(Z3D::ZCameraCalibration *calibration)
    : m_calibration(calibration)
{
    const int w = m_calibration->sensorWidth() - 1.001;
    const int h = m_calibration->sensorHeight() - 1.001;

    setInterval( Qt::XAxis, QwtInterval(0., w) );
    setInterval( Qt::YAxis, QwtInterval(0., h) );

    updateMaxDistortion();

    //        QObject::connect(calibration, SIGNAL(calibrationReady()),
    //                         this, SLOT(updateMaxDistortion()));
}

double ZCalibrationDistortionData::value(double x, double y) const
{
    if (x<0 || y<0 || x>=m_calibration->sensorWidth() || y>=m_calibration->sensorHeight())
        return 0.;

    float cx, cy;
    if (m_calibration->getCalibratedSubPixel(x, y, &cx, &cy)) {
        //qDebug() << "getCalibratedSubPixel executed correctly";

        const float cx_x = cx - x;
        const float cy_y = cy - y;
        float squaredVal = cx_x * cx_x + cy_y * cy_y;

        /// limit distortion to 300px
        if (squaredVal > 90000.f)
            squaredVal = 90000.f;

        double val = sqrt(squaredVal);

//        qDebug() << "sqrt(" << squaredVal << ") = " << val;

        /*
            if (m_maxDistortion < val) {
                m_maxDistortion = val;
                setInterval( Qt::ZAxis, QwtInterval(0., m_maxDistortion) );
            }
    */
        return val;
    } else {
        //qWarning() << "getCalibratedSubPixel failed for x:" << x << "y:" << y;
        return 0.0;
    }
}

void ZCalibrationDistortionData::updateMaxDistortion()
{
#if 1
    const int w = m_calibration->sensorWidth()  - 2;
    const int h = m_calibration->sensorHeight() - 2;

    m_maxDistortion = value(0., 0.);
    for (double ix = 0.0; ix < w; ix += 1.) {
        for (double iy = 0.0; iy < h; iy += 1.) {
            double val = value(ix, iy);
            if (m_maxDistortion < val)
                m_maxDistortion = val;
        }
    }
#else
    const double w = m_calibration->sensorWidth()  - 1.001;
    const double h = m_calibration->sensorHeight() - 1.001;

    /// max distortion is usually on the corners
    m_maxDistortion = value(0., 0.);
    if (m_maxDistortion < value(w, 0.))
        m_maxDistortion = value(w, 0.);
    if (m_maxDistortion < value(w, h))
        m_maxDistortion = value(w, h);
    if (m_maxDistortion < value(0., h))
        m_maxDistortion = value(0., h);
#endif
    setInterval( Qt::ZAxis, QwtInterval(0., m_maxDistortion) );
}

} // namespace Z3D
