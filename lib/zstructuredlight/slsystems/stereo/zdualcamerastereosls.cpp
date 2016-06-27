#include "zdualcamerastereosls.h"

#include "zdualcamerastereoslsconfigwidget.h"
#include "zcalibratedcameraprovider.h"
#include "zpinhole/zpinholecameracalibration.h"

namespace Z3D
{

ZDualCameraStereoSLS::ZDualCameraStereoSLS(QObject *parent)
    : ZStereoSLS(parent)
    , m_configWidget(nullptr)
{

}

ZDualCameraStereoSLS::~ZDualCameraStereoSLS()
{
    qDebug() << "deleting cameras...";
    m_cameras.clear();
}

ZCalibratedCamera::Ptr ZDualCameraStereoSLS::leftCamera() const
{
    return m_cameras[0];
}

ZCalibratedCamera::Ptr ZDualCameraStereoSLS::rightCamera() const
{
    return m_cameras[1];
}

void ZDualCameraStereoSLS::addCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameras)
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

void ZDualCameraStereoSLS::setLeftCamera(ZCalibratedCamera::Ptr camera)
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
                       this, &ZDualCameraStereoSLS::setLeftCalibration);
        }

        m_cameras[0] = camera;

        connect(m_cameras[0].data(), &Z3D::ZCalibratedCamera::calibrationChanged,
                this, &ZDualCameraStereoSLS::setLeftCalibration);

        setLeftCalibration(camera->calibration());
    }
}

void ZDualCameraStereoSLS::setRightCamera(ZCalibratedCamera::Ptr camera)
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
                       this, &ZDualCameraStereoSLS::setRightCalibration);
        }

        m_cameras[1] = camera;

        connect(m_cameras[1].data(), &Z3D::ZCalibratedCamera::calibrationChanged,
                this, &ZDualCameraStereoSLS::setRightCalibration);

        setRightCalibration(camera->calibration());
    }
}

QString ZDualCameraStereoSLS::id() const
{
    return QString("DualCamera");
}

QString ZDualCameraStereoSLS::displayName() const
{
    return QString("Dual cameras");
}

void ZDualCameraStereoSLS::init(QSettings *settings)
{
    QList<ZCalibratedCamera::Ptr> cameras;

    settings->beginGroup("StructuredLightSystem");
    {
        settings->beginGroup("Left");
        {
            cameras << CalibratedCameraProvider::getCalibratedCamera(settings);
        }
        settings->endGroup();

        settings->beginGroup("Right");
        {
            cameras << CalibratedCameraProvider::getCalibratedCamera(settings);
        }
        settings->endGroup();
    }
    settings->endGroup();

    addCameras(cameras);
}

QWidget *ZDualCameraStereoSLS::configWidget()
{
    if (!m_configWidget)
        m_configWidget = new ZDualCameraStereoSLSConfigWidget(this);

    return m_configWidget;
}

void ZDualCameraStereoSLS::onPatternProjected(ZProjectedPattern::Ptr pattern)
{
    Q_UNUSED(pattern);
}

void ZDualCameraStereoSLS::onPatternsDecoded(std::vector<ZDecodedPattern::Ptr> decodedPatterns)
{
    /// is there something to process?
    for (auto decodedPattern : decodedPatterns) {
        if (decodedPattern->estimatedCloudPoints < 1)
            return;
    }

    int estimatedCloudPoints = 0;
    for (const auto &decodedPattern : decodedPatterns)
        estimatedCloudPoints += decodedPattern->estimatedCloudPoints;

    Z3D::ZSimplePointCloud::Ptr cloud = triangulate(
                decodedPatterns[0]->intensityImg,
                decodedPatterns[0]->fringePointsList,
            decodedPatterns[1]->fringePointsList,
            estimatedCloudPoints);

    if (cloud) {
        emit scanFinished(cloud);
    }
}

} // namespace Z3D
