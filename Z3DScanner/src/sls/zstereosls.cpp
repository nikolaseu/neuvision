#include "zstereosls.h"

#include "zstereoslsconfigwidget.h"
#include "zstereosystemimpl.h"
#include "zimageviewer.h"
#include "zcalibratedcameraprovider.h"
#include "zmulticameracalibratorwidget.h"

#include <QFile>
#include <QTimer>

namespace Z3D
{

ZStereoSLS::ZStereoSLS(QObject *parent)
    : ZStructuredLightSystem(parent)
    , m_stereoSystem(nullptr)
    , m_maxValidDistance(0.001)
    , m_debugSaveFringePoints(false)
    , m_debugShowDecodedImages(false)
    , m_debugShowFringes(false)
    , m_calibrationWindow(nullptr)
    , m_configWidget(nullptr)
{
    /// finish initialization
    QTimer::singleShot(0, this, &ZStereoSLS::init);
}

ZStereoSLS::~ZStereoSLS()
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
    m_cameras.clear();
}

QString ZStereoSLS::displayName()
{
    return QString("Stereo");
}

QWidget *ZStereoSLS::getCalibrationWindow()
{
    if (!m_calibrationWindow) {
        m_calibrationWindow = new Z3D::ZMultiCameraCalibratorWidget(m_cameras);
    }

    m_calibrationWindow->show();

    return m_calibrationWindow;
}

double ZStereoSLS::maxValidDistance() const
{
    return m_maxValidDistance;
}

bool ZStereoSLS::debugSaveFringePoints() const
{
    return m_debugSaveFringePoints;
}

bool ZStereoSLS::debugShowDecodedImages() const
{
    return m_debugShowDecodedImages;
}

bool ZStereoSLS::debugShowFringes() const
{
    return m_debugShowFringes;
}

void ZStereoSLS::setMaxValidDistance(double maxValidDistance)
{
    if (m_maxValidDistance == maxValidDistance)
        return;

    m_maxValidDistance = maxValidDistance;
    emit maxValidDistanceChanged(maxValidDistance);
}

void ZStereoSLS::setDebugSaveFringePoints(bool debugSaveFringePoints)
{
    if (m_debugSaveFringePoints == debugSaveFringePoints)
        return;

    m_debugSaveFringePoints = debugSaveFringePoints;
    emit debugSaveFringePointsChanged(debugSaveFringePoints);
}

void ZStereoSLS::setDebugShowDecodedImages(bool debugShowDecodedImages)
{
    if (m_debugShowDecodedImages == debugShowDecodedImages)
        return;

    m_debugShowDecodedImages = debugShowDecodedImages;
    emit debugShowDecodedImagesChanged(debugShowDecodedImages);
}

void ZStereoSLS::setDebugShowFringes(bool debugShowFringes)
{
    if (m_debugShowFringes == debugShowFringes)
        return;

    m_debugShowFringes = debugShowFringes;
    emit debugShowFringesChanged(debugShowFringes);
}

QWidget *ZStereoSLS::configWidget()
{
    if (!m_configWidget)
        m_configWidget = new ZStereoSLSConfigWidget(this);

    return m_configWidget;
}

void ZStereoSLS::init()
{
    m_stereoSystem = new ZStereoSystemImpl();

    connect(m_stereoSystem, &ZStereoSystemImpl::readyChanged,
            this, &ZStereoSLS::setReady);

    /// add configured cameras
    addCameras( CalibratedCameraProvider::loadCameras() );
}

void ZStereoSLS::onPatternsDecoded(std::vector<ZDecodedPattern::Ptr> decodedPatterns)
{
    /// is there something to process?
    for (auto decodedPattern : decodedPatterns) {
        if (decodedPattern->estimatedCloudPoints < 1)
            return;
    }

    if (m_debugShowDecodedImages || m_debugShowFringes) {
        double minVal,
               maxVal,
               absMinVal = DBL_MAX,
               absMaxVal = DBL_MIN;

        /// only to show in the window, we need to change image "range" to improve visibility
        for (const auto &decodedPattern : decodedPatterns) {
            cv::Mat &decoded = decodedPattern->decodedImage;

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
            int iCam = 0;
            for (const auto &decodedPattern : decodedPatterns) {
                cv::Mat &decodedImage = decodedPattern->decodedImage;

                /// convert to "visible" image to show in window
                cv::Mat decodedVisibleImage;
                decodedImage.convertTo(decodedVisibleImage, CV_8U, 255.0/range, -absMinVal * 255.0/range);

                Z3D::ZImageViewer *imageWidget = new Z3D::ZImageViewer();
                imageWidget->setDeleteOnClose(true);
                imageWidget->setWindowTitle(QString("Decoded image [Camera %1]").arg(iCam++));
                imageWidget->updateImage(decodedVisibleImage);
                imageWidget->show();
            }
        }

        if (m_debugShowFringes) {
            int iCam = 0;
            for (const auto &decodedPattern : decodedPatterns) {
                cv::Mat &decodedBorders = decodedPattern->fringeImage;

                /// convert to "visible" image to show in window
                cv::Mat decodedVisibleBorders;
                decodedBorders.convertTo(decodedVisibleBorders, CV_8U, 255.0/range, -absMinVal * 255.0/range);

                Z3D::ZImageViewer *imageWidget = new Z3D::ZImageViewer();
                imageWidget->setDeleteOnClose(true);
                imageWidget->setWindowTitle(QString("Fringe borders [Camera %1]").arg(iCam++));
                imageWidget->updateImage(decodedVisibleBorders);
                imageWidget->show();
            }
        }
    }

    /// we need at least two cameras to continue
    if (!m_stereoSystem)
        return;

    int estimatedCloudPoints = 0;
    for (const auto &decodedPattern : decodedPatterns)
        estimatedCloudPoints += decodedPattern->estimatedCloudPoints;

    Z3D::ZSimplePointCloud::Ptr cloud = m_stereoSystem->triangulateOptimized(
                decodedPatterns[0]->intensityImg,
                decodedPatterns[0]->fringePointsList,
            decodedPatterns[1]->fringePointsList,
            estimatedCloudPoints,
            m_maxValidDistance);

    if (cloud) {
        emit scanFinished(cloud);
    }
}

void ZStereoSLS::addCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameras)
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

    setReady(false);

    /// add cameras to list
    m_cameras.clear();

    if (cameras.size() >= 2) {
        m_cameras.resize(2);
        setLeftCamera(cameras[0]);
        setRightCamera(cameras[1]);
    }

    std::vector<Z3D::ZCameraInterface::Ptr> camerasVector;
    for (auto cam : cameras)
        camerasVector.push_back(cam->camera());
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

void ZStereoSLS::setLeftCamera(ZCalibratedCamera::Ptr camera)
{
    if (m_cameras[0] != camera) {
        /// check first! this only works for pinhole cameras!
        auto *calibration = static_cast<Z3D::ZPinholeCameraCalibration*>(camera->calibration().data());

        if (!calibration) {
            qWarning() << "invalid calibration! this only works for pinhole cameras!";
            return;
        }

        if (m_cameras[0]) {
            disconnect(m_cameras[0].data(), &Z3D::ZCalibratedCamera::calibrationChanged,
                       m_stereoSystem, &ZStereoSystemImpl::setLeftCameraCalibration);
        }

        m_cameras[0] = camera;

        connect(m_cameras[0].data(), &Z3D::ZCalibratedCamera::calibrationChanged,
                m_stereoSystem, &ZStereoSystemImpl::setLeftCameraCalibration);

        m_stereoSystem->setLeftCameraCalibration(camera->calibration());
    }
}

void ZStereoSLS::setRightCamera(ZCalibratedCamera::Ptr camera)
{
    if (m_cameras[1] != camera) {
        /// check first! this only works for pinhole cameras!
        auto *calibration = static_cast<Z3D::ZPinholeCameraCalibration*>(camera->calibration().data());

        if (!calibration) {
            qWarning() << "invalid calibration! this only works for pinhole cameras!";
            return;
        }

        if (m_cameras[1]) {
            disconnect(m_cameras[1].data(), &Z3D::ZCalibratedCamera::calibrationChanged,
                       m_stereoSystem, &ZStereoSystemImpl::setRightCameraCalibration);
        }

        m_cameras[1] = camera;

        connect(m_cameras[1].data(), &Z3D::ZCalibratedCamera::calibrationChanged,
                m_stereoSystem, &ZStereoSystemImpl::setRightCameraCalibration);

        m_stereoSystem->setRightCameraCalibration(camera->calibration());
    }
}

} // namespace Z3D
