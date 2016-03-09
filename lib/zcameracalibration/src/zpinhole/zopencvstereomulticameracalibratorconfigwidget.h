#pragma once

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
