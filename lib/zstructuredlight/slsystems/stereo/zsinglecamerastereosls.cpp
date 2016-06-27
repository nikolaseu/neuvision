#include "zsinglecamerastereosls.h"

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

}

QWidget *ZSingleCameraStereoSLS::configWidget()
{
    return nullptr;
}

void ZSingleCameraStereoSLS::onPatternProjected(ZProjectedPattern::Ptr pattern)
{

}

void ZSingleCameraStereoSLS::onPatternsDecoded(std::vector<ZDecodedPattern::Ptr> patterns)
{

}

} // namespace Z3D
