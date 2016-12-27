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

#ifndef POINTCLOUDWINDOW_H
#define POINTCLOUDWINDOW_H

#include <Z3DLaserTriangulation>
#include <Z3DPointCloud>

#include <pcl/visualization/pcl_visualizer.h>

#include <QMainWindow>
#include <QTime>

namespace Ui {
class PointCloudWindow;
}

class PointCloudWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit PointCloudWindow(QWidget *parent = 0);
    ~PointCloudWindow();
    
    void setCamera(Z3D::LTSCamera3D::WeakPtr camera);

protected slots:
    void showEvent(QShowEvent *);
    void closeEvent(QCloseEvent *);

    void onNewPointCloudReceived(Z3D::ZPointCloud::Ptr cloud);

private:
    Ui::PointCloudWindow *ui;

    Z3D::LTSCamera3D::WeakPtr m_camera3D;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_pclViewer;

    QTime m_elapsedTimeFPS;
    int m_currentCloudIndex;
    QVector<Z3D::ZPointCloud::Ptr> m_currentClouds;
};

#endif // POINTCLOUDWINDOW_H
