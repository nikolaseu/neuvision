#include "zstereostructuredlightsystem.h"

#include "zstereosystem.h"
#include "zimageviewer.h"
#include "zcalibratedcameraprovider.h"
#include "zmulticameracalibratorwidget.h"

#include <QFile>
#include <QTimer>

namespace Z3D
{

ZStereoStructuredLightSystem::ZStereoStructuredLightSystem(QObject *parent)
    : ZStructuredLightSystem(parent)
    , m_stereoSystem(nullptr)
    , m_maxValidDistance(0.001)
    , m_debugSaveFringePoints(false)
    , m_debugShowDecodedImages(false)
    , m_debugShowFringes(false)
{
    /// finish initialization
    QTimer::singleShot(0, this, &ZStereoStructuredLightSystem::init);
}

ZStereoStructuredLightSystem::~ZStereoStructuredLightSystem()
{
    qDebug() << "deleting calibration window...";
    if (m_calibrationWindow)
        m_calibrationWindow->deleteLater();

    qDebug() << "deleting stereo system...";
    if (m_stereoSystem)
        delete m_stereoSystem;

    qDebug() << "deleting cameras...";
//    foreach(Z3D::CalibratedCamera::Ptr camera, m_camList)
//        delete camera.data();
    m_camList.clear();
}

QWidget *ZStereoStructuredLightSystem::getCalibrationWindow()
{
    if (!m_calibrationWindow) {
        m_calibrationWindow = new Z3D::ZMultiCameraCalibratorWidget(m_camList);
    }

    m_calibrationWindow->show();

    return m_calibrationWindow;
}

void ZStereoStructuredLightSystem::init()
{
    /// add configured cameras
    addCameras( Z3D::CalibratedCameraProvider::loadCameras() );
}

void ZStereoStructuredLightSystem::setAcquisitionManager(ZCameraAcquisitionManager *acquisitionManager)
{
    ZStructuredLightSystem::setAcquisitionManager(acquisitionManager);

    setReady(false);

    if (m_camList.size() >= 2 && !m_stereoSystem) {
        m_stereoSystem = new StereoSystem();

        m_stereoSystem->setCamera1(m_camList[0]);
        m_stereoSystem->setCamera2(m_camList[1]);

        m_stereoSystem->stereoRectify();

        connect(m_stereoSystem, &StereoSystem::readyChanged,
                this, &ZStereoStructuredLightSystem::setReady);
    }
}

void ZStereoStructuredLightSystem::onPatternDecoded(ZDecodedPattern::Ptr decodedPattern)
{
    /// is there something to process?
    if (decodedPattern->estimatedCloudPoints < 1)
        return;

    if (m_debugSaveFringePoints) {
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

    if (m_debugShowDecodedImages || m_debugShowFringes) {
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

        if (m_debugShowDecodedImages) {
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

        if (m_debugShowFringes) {
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

    Z3D::ZSimplePointCloud::Ptr cloud = m_stereoSystem->triangulateOptimized(
                decodedPattern->intensityImg,
                decodedPattern->fringePointsList[0],
            decodedPattern->fringePointsList[1],
            decodedPattern->estimatedCloudPoints,
            m_maxValidDistance);

    if (cloud) {
        emit scanFinished(cloud);
    }
}

void ZStereoStructuredLightSystem::addCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameras)
{
    /*
    /// get starting index for the new cameras
    const QList<QAction*> &actionsList = ui->menuCameras->actions();
    int iCam = actionsList.size() - 2; // separator and "configure cameras" items

    if (iCam != m_camList.size()) {
        qCritical() << "something is wrong with the camera indexes";
        return;
    }

    /// the new cameras menu should be inserted before the separator
    QAction *insertBeforeThisAction = actionsList[iCam];
*/

    std::vector<ZCameraInterface::Ptr> camerasVector;

    /// add cameras to list
    m_camList.clear();
    for (auto cam : cameras) {
        m_camList.push_back(cam);
        camerasVector.push_back(cam->camera());
    }

    setAcquisitionManager(new ZCameraAcquisitionManager(camerasVector));

/*
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

    emit cameraListChanged(m_camList);
    */
}

} // namespace Z3D
