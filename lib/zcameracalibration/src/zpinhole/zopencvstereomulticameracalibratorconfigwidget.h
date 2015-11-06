#ifndef Z3D_CAMERACALIBRATOR___ZOPENCVSTEREOMULTICAMERACALIBRATORCONFIGWIDGET_H
#define Z3D_CAMERACALIBRATOR___ZOPENCVSTEREOMULTICAMERACALIBRATORCONFIGWIDGET_H

#include <QWidget>

namespace Ui {
class ZOpenCVStereoMultiCameraCalibratorConfigWidget;
}

namespace Z3D
{

/// fw declaration
class ZOpenCVStereoMultiCameraCalibrator;

class ZOpenCVStereoMultiCameraCalibratorConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ZOpenCVStereoMultiCameraCalibratorConfigWidget(ZOpenCVStereoMultiCameraCalibrator *calibrator, QWidget *parent = 0);
    ~ZOpenCVStereoMultiCameraCalibratorConfigWidget();

private:
    Ui::ZOpenCVStereoMultiCameraCalibratorConfigWidget *ui;

    ZOpenCVStereoMultiCameraCalibrator *m_cameraCalibrator;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZOPENCVSTEREOMULTICAMERACALIBRATORCONFIGWIDGET_H
