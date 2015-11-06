#ifndef Z3D_CAMERACALIBRATOR___ZCALIBRATIONDISTORTIONPLOT_H
#define Z3D_CAMERACALIBRATOR___ZCALIBRATIONDISTORTIONPLOT_H

#include <qwt_plot.h>
#include <qwt_plot_spectrogram.h>

#include <Z3DCameraCalibration>

class QwtPlotRescaler;

namespace Z3D
{

class ZCalibrationDistortionPlot : public QwtPlot
{
    Q_OBJECT

public:
    ZCalibrationDistortionPlot(ZCameraCalibration::Ptr calibration, QWidget *parent = 0);

public slots:
    void showContour(bool on);
    void showSpectrogram(bool on);
    void setAlpha(int);

protected slots:
    void reload();

private:
    QwtPlotSpectrogram *m_spectrogram;

    ZCameraCalibration::Ptr m_calibration;

    QwtPlotRescaler *d_rescaler;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZCALIBRATIONDISTORTIONPLOT_H
