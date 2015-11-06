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
