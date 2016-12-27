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

#pragma once

#include "zlasertriangulation_global.h"
#include "zltscamera3d.h"

#include <QMainWindow>

namespace Z3D
{

namespace Ui {
class ZLTSCameraPreviewer;
}

class LTSProfilePlot;

class Z3D_LASERTRIANGULATION_SHARED_EXPORT LTSCameraPreviewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit LTSCameraPreviewer(Z3D::LTSCamera3D::Ptr camera, QWidget *parent = 0);
    ~LTSCameraPreviewer();

protected slots:
    void closeEvent(QCloseEvent *);

    void onNewImageReceived(Z3D::ZImageGrayscale::Ptr image);
    void onNewPointCloudReceived(Z3D::ZPointCloud::Ptr cloud);

    void onRangeMinimumChanged(double min);
    void onRangeMaximumChanged(double max);
private slots:
    void on_rangeMinSpinBox_valueChanged(double arg1);

    void on_rangeMaxSpinBox_valueChanged(double arg1);

    void on_actionPlayPause_triggered();

    void on_actionAutoSearchROI_triggered();

    void on_actionShowCameraSettings_triggered();

    void on_actionUnsharpMasking_triggered();

    void on_autoSetMinMaxButton_clicked();

    void on_scanViewDisplayModeComboBox_currentIndexChanged(int index);

private:
    Ui::ZLTSCameraPreviewer *ui;

    LTSProfilePlot *m_plot;
    QVector<QPointF> m_samples;

    Z3D::LTSCamera3D::Ptr m_camera;
};

} // namespace Z3D
