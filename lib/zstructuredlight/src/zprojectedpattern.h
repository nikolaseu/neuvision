#pragma once

#include "opencv2/core/core.hpp"

#include "zstructuredlightpattern.h"

#include <QSharedPointer>

#include <map>

namespace Z3D
{

class ZProjectedPattern : public ZStructuredLightPattern
{
public:
    typedef QSharedPointer<ZProjectedPattern> Ptr;

    explicit ZProjectedPattern();
};

} // namespace Z3D
