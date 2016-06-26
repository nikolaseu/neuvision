#include "zstereosls.h"

#include "zstereoslsconfigwidget.h"
#include "zstereosystemimpl.h"
#include "zcalibratedcameraprovider.h"

#include <QFile>
#include <QTimer>

namespace Z3D
{

ZStereoSLS::ZStereoSLS(QObject *parent)
    : ZStructuredLightSystem(parent)
    , m_stereoSystem(nullptr)
    , m_maxValidDistance(0.001)
    , m_configWidget(nullptr)
{
    /// finish initialization
    QTimer::singleShot(0, this, &ZStereoSLS::init);
}

ZStereoSLS::~ZStereoSLS()
{
    qDebug() << "deleting stereo system...";
    if (m_stereoSystem)
        delete m_stereoSystem;

    qDebug() << "deleting cameras...";
    m_cameras.clear();
}

QString ZStereoSLS::displayName()
{
    return QString("Stereo");
}

ZCalibratedCamera::Ptr ZStereoSLS::leftCamera() const
{
    return m_cameras[0];
}

ZCalibratedCamera::Ptr ZStereoSLS::rightCamera() const
{
    return m_cameras[1];
}

double ZStereoSLS::maxValidDistance() const
{
    return m_maxValidDistance;
}

void ZStereoSLS::setMaxValidDistance(double maxValidDistance)
{
    if (m_maxValidDistance == maxValidDistance)
        return;

    m_maxValidDistance = maxValidDistance;
    emit maxValidDistanceChanged(maxValidDistance);
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

void ZStereoSLS::onPatternProjected(ZProjectedPattern::Ptr pattern)
{
    Q_UNUSED(pattern);
}

void ZStereoSLS::onPatternsDecoded(std::vector<ZDecodedPattern::Ptr> decodedPatterns)
{
    /// is there something to process?
    for (auto decodedPattern : decodedPatterns) {
        if (decodedPattern->estimatedCloudPoints < 1)
            return;
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
