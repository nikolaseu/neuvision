#include "zprojectedpattern.h"

#include <QMetaType>

namespace Z3D
{

static int z3dDecodedPatternPtrTypeId = qRegisterMetaType<Z3D::ZProjectedPattern::Ptr>("Z3D::ZProjectedPattern::Ptr");

ZProjectedPattern::ZProjectedPattern()
    : ZStructuredLightPattern()
{

}

} // namespace Z3D
