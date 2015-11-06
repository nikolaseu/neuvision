#ifndef CONFIGURATIONHELPERWINDOW_H
#define CONFIGURATIONHELPERWINDOW_H

#include <Z3DLaserTriangulation>

#include <QMainWindow>

namespace Ui {
class ConfigurationHelperWindow;
}

class ConfigurationHelperWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ConfigurationHelperWindow(Z3D::LTSCamera3D::WeakPtr camera3D, QWidget *parent = 0);
    ~ConfigurationHelperWindow();

private slots:
    void onNewImageReceived(Z3D::ZImageGrayscale::Ptr image);

    void updateValues();

    void on_framePeriodeSpinBox_valueChanged(int arg1);

    void on_multiSlopeModeComboBox_currentIndexChanged(const QString &arg1);

    void on_exposureTimeSpinBox_valueChanged(int arg1);

    void on_dualSlopeTimeSpinBox_valueChanged(int arg1);

    void on_tripleSlopeTimeSpinBox_valueChanged(int arg1);

    void on_thresholdSpinBox_valueChanged(int arg1);

    void on_vLow2SpinBox_valueChanged(int arg1);

    void on_vLow3SpinBox_valueChanged(int arg1);

    void on_vRamp1SpinBox_valueChanged(int arg1);

    void on_vRamp2SpinBox_valueChanged(int arg1);

private:
    void closeEvent(QCloseEvent *);

    Ui::ConfigurationHelperWindow *ui;

    Z3D::LTSCamera3D::WeakPtr m_camera3D;
    Z3D::ZCameraInterface::WeakPtr m_camera;
};

#endif // CONFIGURATIONHELPERWINDOW_H
