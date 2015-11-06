#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <Z3DCalibratedCamera>
#include <Z3DCameraAcquisition>

#include "zcameracalibratorwidget.h"
#include "zmulticameracalibratorwidget.h"

#include "zcloudview.h"
#include "src/sls/threephase.h"
#include "src/sls/stereosystem.h"


#include <pcl/io/pcd_io.h>

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QSignalMapper>
#include <QTimer>





MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_currentPatternProjection(0)
    , m_stereoSystem(0)
    , m_cloudViewer(0)
    , m_calibrationWindow(0)
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

    /// finish initialization
    QTimer::singleShot(0, this, SLOT(init()));
}

MainWindow::~MainWindow()
{
    qDebug() << "deleting pattern projection instances...";
    foreach(BinaryPatternProjection *patternProjection, m_patternProjectionList) {
        delete patternProjection;
    }

    qDebug() << "deleting stereo system...";
    if (m_stereoSystem)
        delete m_stereoSystem;

    qDebug() << "deleting cameras...";
//    foreach(Z3D::CalibratedCamera::Ptr camera, m_camList)
//        delete camera.data();
    m_camList.clear();

    qDebug() << "deleting cloud viewer...";
    if (m_cloudViewer)
        m_cloudViewer->deleteLater();

    qDebug() << "deleting calibration window...";
    if (m_calibrationWindow)
        m_calibrationWindow->deleteLater();

    qDebug() << "deleting ui...";
    delete ui;

    qDebug() << Q_FUNC_INFO << "finished";
}

void MainWindow::init()
{
    /// connect combobox to slot
    QObject::connect(ui->patternTypeComboBox, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(onPatternProjectionTypeChanged(int)));

    /// add types of pattern projection
    m_patternProjectionList << new BinaryPatternProjection();
    ui->patternTypeComboBox->addItem("Binary");

    foreach(BinaryPatternProjection *patternProjection, m_patternProjectionList) {
        /// this are common events, connect all
        QObject::connect(this, SIGNAL(cameraListChanged(QList<Z3D::ZCalibratedCamera::Ptr>)),
                         patternProjection, SLOT(setCameras(QList<Z3D::ZCalibratedCamera::Ptr>)));
        QObject::connect(patternProjection, SIGNAL(patternDecoded(Z3D::DecodedPattern::Ptr)),
                         this, SLOT(onPatternDecoded(Z3D::DecodedPattern::Ptr)));
    }

    /// add configured cameras
    addCameras( Z3D::CalibratedCameraProvider::loadCameras() );
}

void MainWindow::closeEvent(QCloseEvent * /*event*/)
{
    /// closing the main window causes to quit the program
    qApp->quit();
}

Z3D::ZCloudView *MainWindow::getCloudViewer()
{
    if (!m_cloudViewer) {
        m_cloudViewer = new Z3D::ZCloudView();
    }

    m_cloudViewer->show();

    return m_cloudViewer;
}

QWidget *MainWindow::getCalibrationWindow()
{
    if (!m_calibrationWindow) {
        m_calibrationWindow = new Z3D::ZMultiCameraCalibratorWidget(m_camList);
    }

    m_calibrationWindow->show();

    return m_calibrationWindow;
}

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
                            m_currentPatternProjection, SLOT(scan()));
    }

    /// update current pattern finder
    m_currentPatternProjection = m_patternProjectionList[index];

    /// connect events
    /// this is with direct connection because we need to execute this from the gui thread, otherwise sometimes it crashes
    QObject::connect(ui->startAcquisitionButton, SIGNAL(clicked()),
                     m_currentPatternProjection, SLOT(showProjectionWindow()), Qt::DirectConnection);
    QObject::connect(ui->startAcquisitionButton, SIGNAL(clicked()),
                     m_currentPatternProjection, SLOT(scan()));

    /// add config widget
    QWidget *currentWidget = m_currentPatternProjection->configWidget();
    currentWidget->setVisible(true);
    ui->patternProjectionConfigLayout->addWidget(currentWidget);

    /// disable all until ready
    ui->centralwidget->setEnabled(false);
}

void MainWindow::addCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameras)
{
    /// get starting index for the new cameras
    const QList<QAction*> &actionsList = ui->menuCameras->actions();
    int iCam = actionsList.size() - 2; // separator and "configure cameras" items

    if (iCam != m_camList.size()) {
        qCritical() << "something is wrong with the camera indexes";
        return;
    }

    /// the new cameras menu should be inserted before the separator
    QAction *insertBeforeThisAction = actionsList[iCam];

    /// add cameras to list
    m_camList.append(cameras);

    for (int i = iCam; i<m_camList.size(); ++i) {
        Z3D::ZCameraInterface::Ptr camera = m_camList[i]->camera();

        /// add sub menu exclusive for this camera
        QMenu *camSubmenu = new QMenu(camera->uuid(), this);

        /// open preview
        QAction *previewAction = new QAction("Preview...", this);
        camSubmenu->addAction(previewAction);
        QObject::connect(previewAction, SIGNAL(triggered()), m_previewSignalMapper, SLOT(map()));
        m_previewSignalMapper->setMapping(previewAction, i);

        /// open settings
        QAction *settingsAction = new QAction("Settings...", this);
        camSubmenu->addAction(settingsAction);
        QObject::connect(settingsAction, SIGNAL(triggered()), m_settingsSignalMapper, SLOT(map()));
        m_settingsSignalMapper->setMapping(settingsAction, i);

        /// open camera calibration tool
        QAction *openCameraCalibrationAction = new QAction("Calibrate camera...", this);
        camSubmenu->addAction(openCameraCalibrationAction);
        QObject::connect(openCameraCalibrationAction, SIGNAL(triggered()), m_calibrateCameraSignalMapper, SLOT(map()));
        m_calibrateCameraSignalMapper->setMapping(openCameraCalibrationAction, i);

        /// load calibration file
        QAction *loadCalibrationAction = new QAction("Load calibration from file...", this);
        camSubmenu->addAction(loadCalibrationAction);
        QObject::connect(loadCalibrationAction, SIGNAL(triggered()), m_loadCalibrationSignalMapper, SLOT(map()));
        m_loadCalibrationSignalMapper->setMapping(loadCalibrationAction, i);

        /// take 3d snapshot
        QAction *take3dSnapshotAction = new QAction("Show snapshot in 3D...", this);
        camSubmenu->addAction(take3dSnapshotAction);
        QObject::connect(take3dSnapshotAction, SIGNAL(triggered()), m_take3dSnapshotSignalMapper, SLOT(map()));
        m_take3dSnapshotSignalMapper->setMapping(take3dSnapshotAction, i);

        ui->menuCameras->insertMenu(insertBeforeThisAction, camSubmenu);
    }

    if (m_camList.size() >= 2 && !m_stereoSystem) {
        m_stereoSystem = new StereoSystem();

        m_stereoSystem->setCamera1(m_camList[0]);
        m_stereoSystem->setCamera2(m_camList[1]);

        m_stereoSystem->stereoRectify();

        /// activate menu and connect actions
        ui->menu3dReconstruction->setEnabled(true);

        QObject::connect(m_stereoSystem, SIGNAL(readyChanged(bool)),
                         ui->centralwidget, SLOT(setEnabled(bool)));

        QObject::connect(ui->actionUpdateCalibration, SIGNAL(triggered()),
                         m_stereoSystem, SLOT(stereoRectify()));
    } else {
        ui->statusbar->showMessage(tr("3D reconstruction will not be available. Not enough cameras"));
    }

    emit cameraListChanged(m_camList);
}

void MainWindow::openCameraPreview(int camIndex)
{
    Z3D::ZCameraInterface::Ptr pCamera = m_camList[camIndex]->camera();
    Z3D::ZCameraPreviewer *previewDialog = new Z3D::ZCameraPreviewer();
    previewDialog->setCamera(pCamera);
    previewDialog->show();
}

void MainWindow::openCameraSettingsDialog(int camIndex)
{
    Z3D::ZCameraInterface::Ptr pCamera = m_camList[camIndex]->camera();
    pCamera->showSettingsDialog();
}

void MainWindow::openCameraCalibrationWindow(int camIndex)
{
    Z3D::ZCalibratedCamera::Ptr camera = m_camList[camIndex];
    Z3D::ZCameraCalibratorWidget *calibrator = new Z3D::ZCameraCalibratorWidget(camera);
    calibrator->show();
}

void MainWindow::loadCameraCalibrationFile(int camIndex)
{
    Z3D::ZCameraCalibration::Ptr calibration = m_camList[camIndex]->calibration();

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Select camera calibration file"),
                                                    tr("cameracalib.ini"),
                                                    tr("Camera calibration file (*.cameracalib.ini)"));

    if (!fileName.isEmpty() && !fileName.isNull()) {
        if (!calibration->loadFromFile(fileName))
            qCritical() << "calibration could not be loaded";
        //! TODO agregar cartel con mensaje de error
    }
}

void MainWindow::take3dCameraSnapshot(int camIndex)
{
    getCloudViewer()->showCameraSnapshot(m_camList[camIndex]);
}

void MainWindow::onPatternDecoded(Z3D::DecodedPattern::Ptr decodedPattern)
{
    /// is there something to process?
    if (decodedPattern->estimatedCloudPoints < 1)
        return;

    //! TODO: add debug option to enable saving fringe points to file?
    if (false) {
        for (unsigned int iCam=0; iCam<decodedPattern->fringePointsList.size(); iCam++) {

            std::map<int, std::vector<cv::Vec2f> > &fringePoints = decodedPattern->fringePointsList[iCam];
            QFile file(QString("%1/%2.fringePoints.txt").arg(decodedPattern->scanTmpFolder).arg(iCam));
            if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QTextStream fileTextStream(&file);
                fileTextStream << "FringeID PixelX PixelY\n";
                int totalPoints = 0;
                for(std::map<int, std::vector<cv::Vec2f> >::iterator it = fringePoints.begin(); it != fringePoints.end(); ++it) {
                    const std::vector<cv::Vec2f> &pointsVector = it->second;
                    int pointsAmount = pointsVector.size();
                    for (int index = 0; index<pointsAmount; ++index) {
                        const cv::Vec2f &point = pointsVector[index];
                        fileTextStream << it->first << " " << point[0] << " " << point[1] << "\n";
                    }
                }
                fileTextStream.flush();
                file.flush();
                file.close();
                qDebug() << "TOTAL POINTS:" << totalPoints;
            } else {
                qCritical() << "cant open file to write fringePoints" << file.fileName();
            }
        }
    }

    if (ui->debugShowDecodedImagesCheckBox->isChecked() || ui->debugShowFringesCheckBox->isChecked()) {
        double minVal,
               maxVal,
               absMinVal = DBL_MAX,
               absMaxVal = DBL_MIN;

        /// only to show in the window, we need to change image "range" to improve visibility
        for (unsigned int iCam=0; iCam<decodedPattern->fringePointsList.size(); iCam++) {
            cv::Mat &decoded = decodedPattern->decodedImage[iCam];

            ///find minimum and maximum intensities
            minMaxLoc(decoded, &minVal, &maxVal);

            /// minimum will always be zero, we need to find the minimum != 0
            /// we set maxVal where value is zero
            cv::Mat mask = decoded == 0;
            decoded.setTo(cv::Scalar(maxVal), mask);

            /// find correct minimum and maximum intensities (skipping zeros)
            minMaxLoc(decoded, &minVal, &maxVal);

            /// return back to original
            decoded.setTo(cv::Scalar(0), mask);

            if (absMinVal > minVal)
                absMinVal = minVal;
            if (absMaxVal < maxVal)
                absMaxVal = maxVal;
        }

        /// range of values
        double range = (absMaxVal - absMinVal);

        if (ui->debugShowDecodedImagesCheckBox->isChecked()) {
            for (unsigned int iCam=0; iCam<decodedPattern->fringePointsList.size(); iCam++) {
                cv::Mat &decodedImage = decodedPattern->decodedImage[iCam];

                /// convert to "visible" image to show in window
                cv::Mat decodedVisibleImage;
                decodedImage.convertTo(decodedVisibleImage, CV_8U, 255.0/range, -absMinVal * 255.0/range);

                Z3D::ZImageViewer *imageWidget = new Z3D::ZImageViewer();
                imageWidget->setDeleteOnClose(true);
                imageWidget->setWindowTitle(QString("Decoded image [Camera %1]").arg(iCam));
                imageWidget->updateImage(decodedVisibleImage);
                imageWidget->show();
            }
        }

        if (ui->debugShowFringesCheckBox->isChecked()) {
            for (unsigned int iCam=0; iCam<decodedPattern->fringePointsList.size(); iCam++) {
                cv::Mat &decodedBorders = decodedPattern->fringeImage[iCam];

                /// convert to "visible" image to show in window
                cv::Mat decodedVisibleBorders;
                decodedBorders.convertTo(decodedVisibleBorders, CV_8U, 255.0/range, -absMinVal * 255.0/range);

                Z3D::ZImageViewer *imageWidget = new Z3D::ZImageViewer();
                imageWidget->setDeleteOnClose(true);
                imageWidget->setWindowTitle(QString("Fringe borders [Camera %1]").arg(iCam));
                imageWidget->updateImage(decodedVisibleBorders);
                imageWidget->show();
            }
        }
    }

    /// we need at least two cameras to continue
    if (!m_stereoSystem)
        return;

    PointCloudPCLPtr cloud = m_stereoSystem->triangulateOptimized(
                decodedPattern->intensityImg,
                decodedPattern->fringePointsList[0],
            decodedPattern->fringePointsList[1],
            decodedPattern->estimatedCloudPoints,
            ui->maxValidDistanceSpinBox->value());

    if (cloud) {
        /*
        QString filename = QString("%1/pointCloud.pcd").arg(decodedPattern->scanTmpFolder);
        qDebug() << "saving point cloud data to" << filename;
        pcl::io::savePCDFile(qPrintable(filename), *cloud);
        */
        getCloudViewer()->addPointCloud(cloud);
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
    getCloudViewer()->raise();
}

void MainWindow::on_actionQuit_triggered()
{
    qApp->quit();
}

void MainWindow::on_actionCalibrateSystem_triggered()
{
    getCalibrationWindow()->raise();
}

void MainWindow::on_actionShowSnapshot3D_triggered()
{
    //! TODO esto no deberia estar funcionando asi, tengo que organizarlo mejor
    for (int k=0; k<2; k++) {
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
    }
}
