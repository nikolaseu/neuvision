#include "zstructuredlightpattern.h"

namespace Z3D
{

ZStructuredLightPattern::ZStructuredLightPattern()
    : estimatedCloudPoints(0)
{

}

void ZStructuredLightPattern::updatePointCount()
{
    estimatedCloudPoints = 0;
    for (auto it = fringePointsList.cbegin(), itEnd = fringePointsList.cend(); it != itEnd; ++it) {
        estimatedCloudPoints += it->second.size();
    }
}

} // namespace Z3D
