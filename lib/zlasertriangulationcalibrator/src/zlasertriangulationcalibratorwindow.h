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

#include "zlasertriangulationcalibrator_global.h"

#include "zcalibrationpatternfinder.h"

#include <Z3DCalibratedCamera>

#include <QMainWindow>
#include <QModelIndex>

namespace Ui {
class ZLaserTriangulationCalibratorWindow;
}

class QProgressBar;

namespace Z3D
{

class ZLTSCalibrationImageModel;
class ZLaserTriangulationCalibrator;

class Z3D_LASERTRIANGULATIONCALIBRATOR_SHARED_EXPORT ZLaserTriangulationCalibratorWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ZLaserTriangulationCalibratorWindow(ZCalibratedCamera::WeakPtr camera3D, QWidget *parent = 0);
    ~ZLaserTriangulationCalibratorWindow();

private slots:
    virtual void closeEvent(QCloseEvent *event);

    void onCurrentSelectionChanged(QModelIndex current, QModelIndex previous);
    void onCalibrationPatternTypeChanged(int index);

    void onProgressChanged(float progress, QString message);
    void onCalibrationChanged();

    void on_pushButton_clicked();

    void on_imageViewPageButton_clicked();

    void on_cameraViewPageButton_clicked();

    void on_calibrationPageButton_clicked();

    void on_runCalibrationButton_clicked();

    void on_imageViewTypeComboBox_currentIndexChanged(int index);

    void on_calibrationPatternViewButton_clicked();

private:
    Ui::ZLaserTriangulationCalibratorWindow *ui;
    QProgressBar *m_statusProgressBar;

    ZCalibratedCamera::WeakPtr m_camera;
    //Z3D::LTSCamera3D::WeakPtr m_camera3D;

    ZLTSCalibrationImageModel *m_model;

    QList<ZCalibrationPatternFinder::Ptr> m_patternFinderList;
    ZCalibrationPatternFinder::Ptr m_currentPatternFinder;

    ZLaserTriangulationCalibrator *m_ltsCalibrator;
};

} // namespace Z3D
