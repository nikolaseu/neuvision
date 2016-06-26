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

QString ZSingleCameraStereoSLS::displayName()
{
    return QString("Projector + Camera");
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
