#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <Z3DCalibratedCamera>
#include <Z3DCameraAcquisition>

#include "src/sls/zstereosls.h"
//#include "zcloudview.h"
#include "src/sls/zbinarypatternprojection.h"


//#include <pcl/io/pcd_io.h>

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QMessageBox>
#include <QSignalMapper>
#include <QTimer>





MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_currentPatternProjection(nullptr)
    , m_structuredLightSystem(nullptr)
    /// FIXME , m_cloudViewer(nullptr)
{
    ui->setupUi(this);

    /// start disabled. only enabled when we have at least 2 cameras
    ui->menu3dReconstruction->setEnabled(false);

    /// camera menu
    QAction *configureCamerasAction = new QAction("Configure cameras...", this);
    configureCamerasAction->setEnabled(false);
    //QObject::connect(configureCamerasAction, SIGNAL(triggered()), this, SLOT());
    ui->menuCameras->addSeparator();
    ui->menuCameras->addAction(configureCamerasAction);

    /// signal mappers to use only one function for each action, the camera will be obtained using the index
    m_previewSignalMapper = new QSignalMapper(this);
    QObject::connect(m_previewSignalMapper, SIGNAL(mapped(int)),
                     this, SLOT(openCameraPreview(int)));

    m_settingsSignalMapper = new QSignalMapper(this);
    QObject::connect(m_settingsSignalMapper, SIGNAL(mapped(int)),
                     this, SLOT(openCameraSettingsDialog(int)));

    m_calibrateCameraSignalMapper = new QSignalMapper(this);
    QObject::connect(m_calibrateCameraSignalMapper, SIGNAL(mapped(int)),
                     this, SLOT(openCameraCalibrationWindow(int)));

    m_loadCalibrationSignalMapper = new QSignalMapper(this);
    QObject::connect(m_loadCalibrationSignalMapper, SIGNAL(mapped(int)),
                     this, SLOT(loadCameraCalibrationFile(int)));

    m_take3dSnapshotSignalMapper = new QSignalMapper(this);
    QObject::connect(m_take3dSnapshotSignalMapper, SIGNAL(mapped(int)),
                     this, SLOT(take3dCameraSnapshot(int)));

    /// disable all until ready
    ui->centralwidget->setEnabled(false);

    /// finish initialization
    QTimer::singleShot(0, this, &MainWindow::init);
}

MainWindow::~MainWindow()
{
    qDebug() << "deleting pattern projection instances...";
    foreach(Z3D::ZPatternProjection *patternProjection, m_patternProjectionList) {
        delete patternProjection;
    }

    qDebug() << "deleting cloud viewer...";
    /// FIXME
    /// if (m_cloudViewer)
    ///     m_cloudViewer->deleteLater();

    qDebug() << "deleting ui...";
    delete ui;

    qDebug() << Q_FUNC_INFO << "finished";
}

void MainWindow::init()
{
    m_structuredLightSystem = new Z3D::ZStereoSLS();
    connect(m_structuredLightSystem, &Z3D::ZStructuredLightSystem::readyChanged,
            ui->centralwidget, &QWidget::setEnabled);
    connect(m_structuredLightSystem, &Z3D::ZStructuredLightSystem::scanFinished,
            this, &MainWindow::onScanFinished);
    connect(ui->startAcquisitionButton, &QCommandLinkButton::clicked,
            m_structuredLightSystem, &Z3D::ZStructuredLightSystem::start);

    ui->structuredLightSystemComboBox->addItem(m_structuredLightSystem->displayName());

    /// add config widget
    QWidget *currentWidget = m_structuredLightSystem->configWidget();
    ui->structuredLightSystemConfigLayout->addWidget(currentWidget);
    currentWidget->setVisible(true);

    /// connect combobox to slot
    connect(ui->patternTypeComboBox, SIGNAL(currentIndexChanged(int)),
            this, SLOT(onPatternProjectionTypeChanged(int)));

    /// add types of pattern projection
    Z3D::ZPatternProjection *binaryPatternProjection = new Z3D::ZBinaryPatternProjection();
    m_patternProjectionList << binaryPatternProjection;
    ui->patternTypeComboBox->addItem(binaryPatternProjection->displayName());
}

void MainWindow::closeEvent(QCloseEvent * /*event*/)
{
    /// closing the main window causes the program to quit
    qApp->quit();
}

/// FIXME
//Z3D::ZCloudView *MainWindow::getCloudViewer()
//{
//    if (!m_cloudViewer) {
//        m_cloudViewer = new Z3D::ZCloudView();
//    }

//    m_cloudViewer->show();

//    return m_cloudViewer;
//}

void MainWindow::onPatternProjectionTypeChanged(int index)
{
    /// remove previous config widget and hide it
    if (m_currentPatternProjection) {
        QWidget *previousWidget = m_currentPatternProjection->configWidget();
        previousWidget->setVisible(false);
        ui->patternProjectionConfigLayout->removeWidget(previousWidget);

        QObject::disconnect(ui->startAcquisitionButton, SIGNAL(clicked()),
                            m_currentPatternProjection, SLOT(showProjectionWindow()));
        QObject::disconnect(ui->startAcquisitionButton, SIGNAL(clicked()),
                            m_currentPatternProjection, SLOT(beginScan()));
    }

    /// update current pattern finder
    m_currentPatternProjection = m_patternProjectionList[index];
    m_structuredLightSystem->setPatternProjection(m_currentPatternProjection);

    /// add config widget
    QWidget *currentWidget = m_currentPatternProjection->configWidget();
    ui->patternProjectionConfigLayout->addWidget(currentWidget);
    currentWidget->setVisible(true);
}

void MainWindow::openCameraPreview(int camIndex)
{
    /*Z3D::ZCameraInterface::Ptr pCamera = m_camList[camIndex]->camera();
    Z3D::ZCameraPreviewer *previewDialog = new Z3D::ZCameraPreviewer();
    previewDialog->setCamera(pCamera);
    previewDialog->show();*/
}

void MainWindow::openCameraSettingsDialog(int camIndex)
{
    /*Z3D::ZCameraInterface::Ptr pCamera = m_camList[camIndex]->camera();
    pCamera->showSettingsDialog();*/
}

void MainWindow::openCameraCalibrationWindow(int camIndex)
{
    /*Z3D::ZCalibratedCamera::Ptr camera = m_camList[camIndex];
    Z3D::ZCameraCalibratorWidget *calibrator = new Z3D::ZCameraCalibratorWidget(camera);
    calibrator->show();*/
}

void MainWindow::loadCameraCalibrationFile(int camIndex)
{
    /*Z3D::ZCameraCalibration::Ptr calibration = m_camList[camIndex]->calibration();

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Select camera calibration file"),
                                                    tr("cameracalib.ini"),
                                                    tr("Camera calibration file (*.cameracalib.ini)"));

    if (!fileName.isEmpty() && !fileName.isNull()) {
        if (!calibration->loadFromFile(fileName))
            qCritical() << "calibration could not be loaded";
        //! TODO agregar cartel con mensaje de error
    }*/
}

void MainWindow::take3dCameraSnapshot(int camIndex)
{
    //getCloudViewer()->showCameraSnapshot(m_camList[camIndex]);
}

void MainWindow::onScanFinished(Z3D::ZSimplePointCloud::Ptr cloud)
{
    if (!cloud) {
        QMessageBox::warning(this, tr("Scan error"),
                             tr("The point cloud is empty.\nIs everything configured correctly?\nIs there anything inside the scan volume?"));
        return;
    }

    /*
    QString filename = QString("%1/pointCloud.pcd").arg(decodedPattern->scanTmpFolder);
    qDebug() << "saving point cloud data to" << filename;
    pcl::io::savePCDFile(qPrintable(filename), *cloud);
    */
    /// FIXME
    /// getCloudViewer()->addPointCloud(cloud);


    QDir folder = QDir::current();
#if defined(Q_OS_MAC)
    if (folder.dirName() == "MacOS") {
        folder.cdUp();
        folder.cdUp();
        folder.cdUp();
    }
#endif

    QString fileName = QFileDialog::getSaveFileName(
                this,
                tr("Save point cloud"),
                QString("%1/%2_pointCloud.asc")
                    .arg(folder.absolutePath())
                    .arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmss")),
                tr("ASCII file (*.asc);;Point Cloud Library file (*.pcd)"));

    if (fileName.isEmpty() || fileName.isNull()) {
        return;
    }

    QFile file(fileName);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream fileTextStream(&file);
        if (fileName.endsWith(".pcd", Qt::CaseInsensitive)) {
            /// if trying to save to PCD format
            fileTextStream << QString(
                              "# .PCD v.7 - Point Cloud Data file format\n"
                              "VERSION .7\n"
                              "FIELDS x y z rgb\n"
                              "SIZE 4 4 4 4\n"
                              "TYPE F F F F\n"
                              "COUNT 1 1 1 1\n"
                              "WIDTH %1\n"
                              "HEIGHT 1\n"
                              "VIEWPOINT 0 0 0 1 0 0 0\n"
                              "POINTS %1\n"
                              "DATA ascii\n")
                              .arg(cloud->points.size());
            for (const auto &point : cloud->points) {
                fileTextStream << point[0] << " " << point[1] << " " << point[2] << " " << point[3] << "\n";
            }
        } else if (fileName.endsWith(".asc", Qt::CaseInsensitive)) {
            /// ASCII file format
            for (const auto &point : cloud->points) {
                fileTextStream << point[0] << " " << point[1] << " " << point[2] << " " << point[3] << "\n";
            }
        } else {
            QMessageBox::warning(this, tr("Error saving point cloud"),
                                 tr("Invalid file %1\nNothing saved.").arg(fileName));
        }
        fileTextStream.flush();
        file.flush();
        file.close();
        qDebug() << "point cloud written to" << file.fileName();
    } else {
        qCritical() << "cant open file to write fringePoints" << file.fileName();
    }
}

void MainWindow::getThreePhasePatternPhotos()
{
    //! FIXME pasar esto a su propia clase "ThreePhasePatternProjection"?
//    int imageCnt = 0;

//    cv::Mat image;
//    std::vector<cv::Mat> images;
//    images.reserve(3);

//    Z3D::CameraInterface::Ptr cam = m_camList[1]->camera();
//    cam->startAcquisition();

//    while ( imageCnt < 3 ) {
//        // Create a unique filename
//        char filename[512];
//        sprintf( filename, "%d.png", imageCnt*120 );

//        imageCnt++;
//        printf( "Grabbed image %d\n", imageCnt );

//        ui->phaseXDoubleSpinBox->setValue((imageCnt%3) * 120);

//        QCoreApplication::processEvents();

//#if QT_VERSION < 0x050000
//        SleeperThread::msleep(ui->interPhotoDelaySpinBox->value());
//#else
//        QThread::msleep(ui->interPhotoDelaySpinBox->value());
//#endif

//        QCoreApplication::processEvents();

//        // Get images
//        image = Z3D::ImageUtils::toCVMat( cam->getSnapshot() );
//        images.push_back(image.clone()); // clone just in case. is really neccessary?

//        if (ui->debugShowDecodedImagesCheckBox->isChecked()) {
//            // display images
//            qCVImageWidget *imageWidget = new qCVImageWidget();
//            imageWidget->setWindowTitle(QString("Image %1 [%2]").arg(imageCnt).arg(cam->uuid()));
//            imageWidget->setImage(image);
//            imageWidget->show();
//        }
//    }

//    cam->stopAcquisition();

//    int ncols = image.cols;
//    int nrows = image.rows;

//    ThreePhase tp(ncols, nrows);
//    tp.setNoiseThreshold(ui->noiseThresholdSpinBox->value());
////    tp.setMakeDepth(ui->makeDepthCheckBox->isChecked());
//    tp.calculateDepth(images[0], images[1], images[2]);

//    /*
//    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud = tp.point_cloud_ptr;
//    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> rgb(cloud);
//    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> rgb(cloud, "intensity");
//    viewer->removePointCloud("sample cloud");
//    viewer->addPointCloud<pcl::PointXYZI> (cloud, rgb, "sample cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
//*/

//    if (ui->debugShowDecodedImagesCheckBox->isChecked()) {
//        QLabel *imageWidget = new QLabel();
//        imageWidget->setWindowTitle("Wrapped phase");
//        imageWidget->setPixmap(QPixmap::fromImage(QImage(tp.wrappedPhase, ncols, nrows, QImage::Format_RGB888)));
//        imageWidget->show();

//        QLabel *imageWidget2 = new QLabel();
//        imageWidget2->setWindowTitle("Unwrapped phase");
//        imageWidget2->setPixmap(QPixmap::fromImage(QImage(tp.unwrappedPhase, ncols, nrows, QImage::Format_RGB888)));
//        imageWidget2->show();

//        /*QLabel *imageWidget3 = new QLabel();
//        imageWidget3->setWindowTitle("Color position phase");
//        imageWidget3->setPixmap(QPixmap::fromImage(QImage(tp.colorPositionImage, ncols, nrows, QImage::Format_RGB888)));
//        imageWidget3->show();*/
//    }
}

void MainWindow::on_actionCloudViewer_triggered()
{
    /// FIXME getCloudViewer()->raise();
}

void MainWindow::on_actionQuit_triggered()
{
    qApp->quit();
}

void MainWindow::on_actionShowSnapshot3D_triggered()
{
    //! TODO esto no deberia estar funcionando asi, tengo que organizarlo mejor
    /*for (int k=0; k<2; k++) {
        qDebug() << "obtaining rectified snapshot for camera" << k << "...";
        PointCloudPCLPtr cloud = m_stereoSystem->getRectifiedSnapshot3D(k, 80, 4);
        qDebug() << "removing previous cloud for rectified snapshot for camera" << k << "...";
        getCloudViewer()->getViewer()->removePointCloud( qPrintable(QString("rectifiedSnapshot_%1").arg(k)) );
        qDebug() << "adding cloud for rectified snapshot for camera" << k << "...";
        //pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(cloud);
        pcl::visualization::PointCloudColorHandlerGenericField<PointType> rgb(cloud, "intensity");
        getCloudViewer()->getViewer()->addPointCloud<PointType>(cloud, rgb, qPrintable(QString("rectifiedSnapshot_%1").arg(k)) );
        qDebug() << "drawing frustrum for camera" << k << "...";
        getCloudViewer()->drawCameraFrustrum(m_camList[k], 500);
    }*/
}
