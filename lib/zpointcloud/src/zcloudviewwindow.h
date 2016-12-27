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

#include <QMainWindow>

#include <Z3DCalibratedCamera>
#include <Z3DPointCloud>

// hack to avoid "macro argument mismatch" with some boost lib and Qt5
#ifndef Q_MOC_RUN
#include <pcl/visualization/pcl_visualizer.h>
#endif // Q_MOC_RUN

namespace Z3D
{

namespace Ui {
class ZCloudViewWindow;
}

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZCloudViewWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ZCloudViewWindow(QWidget *parent = 0);
    ~ZCloudViewWindow();

    void showCameraSnapshot(Z3D::ZCalibratedCamera::Ptr pCamera);

    void drawCameraFrustrum(Z3D::ZCalibratedCamera::Ptr cam, float scale);

    void addPointCloud(PointCloudPCLPtr cloud, const QString &id = QString("cloud"));

    boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer();

private slots:
    void on_actionLoadPointCloud_triggered();

    void on_actionSavePointCloudAs_triggered();

    void on_actionCloseAllPointClouds_triggered();

    void on_actionToolsStatisticalOutlierRemoval_triggered();

    void on_actionToolsMovingLeastSquares_triggered();

    void on_actionToolsGreedyTriangulation_triggered();

    void on_actionToolsGridProjection_triggered();

    void on_actionDownsample_triggered();

    void on_actionViewCoordinateSystem_toggled(bool arg1);

    void on_actionToolsFitCylinder_triggered();

    void on_actionToolsFitPlane_triggered();

    void on_actionViewNormals_toggled(bool arg1);

    void on_actionViewPoints_toggled(bool arg1);

    void on_actionViewSurface_toggled(bool arg1);

    void on_actionChangeBackgroundColor_triggered();

    void on_actionRenderShadingFlat_triggered();

    void on_actionRenderShadingGouraud_triggered();

    void on_actionRenderShadingPhong_triggered();

    void on_actionRenderToFile_triggered();

    void on_actionToolsDelaunay2D_triggered();

private:
    Ui::ZCloudViewWindow *ui;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_pclViewer;

    Z3D::ZPointCloud::Ptr m_pointCloud;
    //SurfaceElementsPtr m_surfaceElements;

    QColor m_currentBackgroundColor;
};

} // namespace Z3D
