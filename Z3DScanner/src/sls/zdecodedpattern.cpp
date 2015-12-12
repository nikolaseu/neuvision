#include "zdecodedpattern.h"

#include <QMetaType>

namespace Z3D
{

static int z3dDecodedPatternPtrTypeId = qRegisterMetaType<Z3D::ZDecodedPattern::Ptr>("Z3D::DecodedPattern::Ptr");

ZDecodedPattern::ZDecodedPattern(int numOfCameras) :
    estimatedCloudPoints(0)
{
    fringePointsList.resize(numOfCameras);
    maskImg.resize(numOfCameras);
    decodedImage.resize(numOfCameras);
    fringeImage.resize(numOfCameras);
}

} // namespace Z3D
