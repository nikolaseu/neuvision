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
}

QWidget *ZSingleCameraStereoSLS::configWidget()
{
    return new QLabel("TO DO");
}

void ZSingleCameraStereoSLS::onPatternProjected(ZProjectedPattern::Ptr pattern)
{

}

void ZSingleCameraStereoSLS::onPatternsDecoded(std::vector<ZDecodedPattern::Ptr> patterns)
{

}

} // namespace Z3D
