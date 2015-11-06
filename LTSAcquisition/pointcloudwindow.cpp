#include "pointcloudwindow.h"
#include "ui_pointcloudwindow.h"

#include <QDebug>
#include <QTimer>

#ifdef Q_OS_LINUX
#include "vtkCamera.h" // necessary in linux
//#include "vtkLegendScaleActor.h"
#include "vtkRenderWindow.h" // necessary in linux
#endif

PointCloudWindow::PointCloudWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PointCloudWindow),
    m_pclViewer(new pcl::visualization::PCLVisualizer("3D", false)),
    m_currentCloudIndex(0)
{
    qDebug() << Q_FUNC_INFO;

    ui->setupUi(this);

    //! Creo qvtkwidget (visualizador)
    vtkSmartPointer<vtkRenderWindow> renderWindow = m_pclViewer->getRenderWindow();
    ui->qvtkWidget->SetRenderWindow(renderWindow);

    // these are useful to add to make the controls more like pcd_viewer
    m_pclViewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    m_pclViewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);

    m_pclViewer->setBackgroundColor(0.2, 0.2, 0.2);
    m_pclViewer->initCameraParameters();

    // set orthographic projection
    vtkSmartPointer<vtkRendererCollection> rendererCollection = m_pclViewer->getRenderWindow()->GetRenderers();
    rendererCollection->InitTraversal();
    vtkSmartPointer<vtkRenderer> renderer = rendererCollection->GetNextItem();
    vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
    camera->ParallelProjectionOn();
    camera->SetParallelScale(1000);
    renderer->Render();
}

PointCloudWindow::~PointCloudWindow()
{
    qDebug() << Q_FUNC_INFO;

    delete ui;
}

void PointCloudWindow::setCamera(Z3D::LTSCamera3D::WeakPtr camera)
{
    if (m_camera3D != camera) {
        if (m_camera3D) {
            /// disconnect signals from camera (it should be automatic, but anyway...)
            QObject::disconnect(m_camera3D.data(), SIGNAL(newPointCloudReceived(Z3D::ZPointCloud::Ptr)),
                                this, SLOT(onNewPointCloudReceived(Z3D::ZPointCloud::Ptr)));
        }

        m_camera3D = camera;
    }
}

void PointCloudWindow::showEvent(QShowEvent *)
{
    qDebug() << Q_FUNC_INFO;

    if (m_camera3D) {
        /// disconnect signals from camera (it should be automatic, but anyway...)
        QObject::connect(m_camera3D.data(), SIGNAL(newPointCloudReceived(Z3D::ZPointCloud::Ptr)),
                         this, SLOT(onNewPointCloudReceived(Z3D::ZPointCloud::Ptr)));
    }
}

void PointCloudWindow::closeEvent(QCloseEvent *)
{
    qDebug() << Q_FUNC_INFO;

    if (m_camera3D) {
        /// disconnect signals from camera (it should be automatic, but anyway...)
        QObject::disconnect(m_camera3D.data(), SIGNAL(newPointCloudReceived(Z3D::ZPointCloud::Ptr)),
                            this, SLOT(onNewPointCloudReceived(Z3D::ZPointCloud::Ptr)));
    }
}

void PointCloudWindow::onNewPointCloudReceived(Z3D::ZPointCloud::Ptr cloud)
{
    bool unlimitedFPS = false;

    if (ui->viewerMaxFramerateSpinBox->value() < 1)
        unlimitedFPS = true;

    const int maxVisibleClouds = ui->viewerMaxCloudNumberSpinBox->value();

    /// just created, we must start it
    if (m_elapsedTimeFPS.isNull())
        m_elapsedTimeFPS.start();

    if (m_currentClouds.size() < maxVisibleClouds) {
        m_currentClouds.push_back(cloud);

        m_currentCloudIndex = m_currentClouds.size() - 1;

        //pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(cloud);
        pcl::visualization::PointCloudColorHandlerGenericField<PointType> rgb(m_currentClouds[m_currentCloudIndex]->pclPointCloud(), "y");
        m_pclViewer->addPointCloud<PointType>(m_currentClouds[m_currentCloudIndex]->pclPointCloud(), rgb,
                                          qPrintable(QString("cloud_%1").arg(m_currentCloudIndex)));

        ui->qvtkWidget->update();
    } else {
        // update maximum X times per second
        if (unlimitedFPS || m_elapsedTimeFPS.elapsed() > (1000.f / ui->viewerMaxFramerateSpinBox->value())) {
            m_elapsedTimeFPS.restart();

            QTime updateTime;
            updateTime.start();

            m_currentCloudIndex++;

            if (m_currentCloudIndex >= maxVisibleClouds) {
                m_currentCloudIndex = 0;
            }

            m_currentClouds[m_currentCloudIndex] = cloud;

            //pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(cloud);
            pcl::visualization::PointCloudColorHandlerGenericField<PointType> rgb(cloud->pclPointCloud(), "y");
            m_pclViewer->updatePointCloud<PointType>(cloud->pclPointCloud(), rgb,
                                                 qPrintable(QString("cloud_%1").arg(m_currentCloudIndex)));

            /*std::vector<pcl::visualization::Camera> cameras;
            mViewer->getCameras(cameras);
            pcl::visualization::Camera &camera1 = cameras[0];
            qDebug() << camera1.pos[0] << camera1.pos[1] << camera1.pos[2];*/

//            mViewer->resetCamera();

            //ui->qvtkWidget->update();

//            qDebug() << "updateTime:" << updateTime.elapsed() << "msecs";
        }
    }
}
