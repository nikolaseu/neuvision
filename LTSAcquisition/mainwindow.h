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

#ifndef Z3D___MAINWINDOW_H
#define Z3D___MAINWINDOW_H

#include <QMainWindow>
#include <QStringListModel>
//#include <QThread>

#include "zltsthreadedcamera.h"
//#include "cameraimage.h"
//#include "cameraframesrecorder.h"
//#include "ltscamera3d.h"
//#include "pointcloud_typedefs.h"
//#include "pointcloud.h"
//#include "pointcloudrecorder.h"

class PointCloudWindow;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
signals:
    void connectedChanged(bool connected);

private slots:
    void onCurrentSelectionChanged(QModelIndex current, QModelIndex previous);

    void on_connectButton_clicked();

    void on_acqModeComboBox_currentIndexChanged(int index);

    void on_open3DViewButton_clicked();

    void on_openImageViewButton_clicked();

    void on_startAcqButton_clicked();

    void on_stopAcqButton_clicked();

    void on_frameRecorderEnabledCheckBox_toggled(bool checked);

    void on_cloudRecorderEnabledCheckBox_toggled(bool checked);

    void on_calibrateLTSButton_clicked();

    void on_cameraSettingsHelperButton_clicked();

    void on_openCameraSettingsButton_clicked();

    void on_calibrateCameraButton_clicked();

    void on_addCameraButton_clicked();

private:
    Ui::MainWindow *ui;

    PointCloudWindow *m_3dview;

    QStringListModel m_cameraListModel;

    QList<ZLTSThreadedCamera::Ptr> m_camera3DList;
    ZLTSThreadedCamera::Ptr m_selectedCamera3D;
};

#endif // Z3D___MAINWINDOW_H
