#pragma once

#include <QWidget>

namespace Ui {
class ZPinholeCameraCalibratorConfigWidget;
}

namespace Z3D
{

/// fw declaration
class ZPinholeCameraCalibrator;

class ZPinholeCameraCalibratorConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ZPinholeCameraCalibratorConfigWidget(ZPinholeCameraCalibrator *calibrator, QWidget *parent = 0);
    ~ZPinholeCameraCalibratorConfigWidget();

private:
    Ui::ZPinholeCameraCalibratorConfigWidget *ui;

    ZPinholeCameraCalibrator *m_cameraCalibrator;
};

} // namespace Z3D
