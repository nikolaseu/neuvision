/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

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
