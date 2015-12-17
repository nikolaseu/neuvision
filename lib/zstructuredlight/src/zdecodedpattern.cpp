#include "zdecodedpattern.h"

#include <QMetaType>

namespace Z3D
{

static int z3dDecodedPatternPtrTypeId = qRegisterMetaType<Z3D::ZDecodedPattern::Ptr>("Z3D::DecodedPattern::Ptr");

ZDecodedPattern::ZDecodedPattern()
    : estimatedCloudPoints(0)
{

}

void ZDecodedPattern::updatePointCount()
{
    estimatedCloudPoints = 0;
    for (auto it = fringePointsList.cbegin(), itEnd = fringePointsList.cend(); it != itEnd; ++it) {
        estimatedCloudPoints += it->second.size();
    }
}

} // namespace Z3D
