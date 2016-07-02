#include "zsinglecamerastereosls.h"

#include "zcameracalibrationprovider.h"
#include "zcalibratedcameraprovider.h"

#include <QLabel>

namespace Z3D
{

ZSingleCameraStereoSLS::ZSingleCameraStereoSLS(QObject *parent)
    : ZStereoSLS(parent)
{

}

ZSingleCameraStereoSLS::~ZSingleCameraStereoSLS()
{

}

void ZSingleCameraStereoSLS::processPatterns()
{
    /// is there something to process?
    if (!projectedPattern || decodedPatterns.empty()) {
        return;
    }

    int estimatedCloudPoints = projectedPattern->estimatedCloudPoints;
    for (const auto &decodedPattern : decodedPatterns)
        estimatedCloudPoints += decodedPattern->estimatedCloudPoints;

    Z3D::ZSimplePointCloud::Ptr cloud = triangulate(
                decodedPatterns[0]->intensityImg,
                decodedPatterns[0]->fringePointsList,
            projectedPattern->fringePointsList,
            estimatedCloudPoints);

    if (cloud) {
        emit scanFinished(cloud);
    }

    projectedPattern = ZProjectedPattern::Ptr(nullptr);
    decodedPatterns.clear();
}

QString ZSingleCameraStereoSLS::id() const
{
    return QString("Projector+Camera");
}

QString ZSingleCameraStereoSLS::displayName() const
{
    return QString("Projector + Camera");
}

void ZSingleCameraStereoSLS::init(QSettings *settings)
{
    camera = CalibratedCameraProvider::getCalibratedCamera(settings);

    settings->beginGroup("ProjectorCalibration");
    {
        projectorCalibration =  ZCameraCalibrationProvider::getCalibration(settings);
    }
    settings->endGroup();

    setLeftCalibration(camera->calibration());
    setRightCalibration(projectorCalibration);

    std::vector<Z3D::ZCameraInterface::Ptr> camerasVector;
    camerasVector.push_back(camera->camera());
    setAcquisitionManager(new ZCameraAcquisitionManager(camerasVector));
}

QWidget *ZSingleCameraStereoSLS::configWidget()
{
    return new QLabel("TO DO");
}

void ZSingleCameraStereoSLS::onPatternProjected(ZProjectedPattern::Ptr pattern)
{
    if (pattern && pattern->estimatedCloudPoints > 0) {
        projectedPattern = pattern;
        processPatterns();
    } else {
        decodedPatterns.clear();
    }
}

void ZSingleCameraStereoSLS::onPatternsDecoded(std::vector<ZDecodedPattern::Ptr> patterns)
{
    for (auto decodedPattern : patterns) {
        if (decodedPattern->estimatedCloudPoints < 1) {
            projectedPattern = ZProjectedPattern::Ptr(nullptr);
            return;
        }
    }

    decodedPatterns = patterns;
    processPatterns();
}

} // namespace Z3D
