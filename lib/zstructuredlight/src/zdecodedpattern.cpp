#include "zdecodedpattern.h"

#include <QMetaType>

namespace Z3D
{

static int z3dDecodedPatternPtrTypeId = qRegisterMetaType<Z3D::ZDecodedPattern::Ptr>("Z3D::DecodedPattern::Ptr");

ZDecodedPattern::ZDecodedPattern()
    : ZStructuredLightPattern()
{

}

} // namespace Z3D
