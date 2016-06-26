#pragma once

#include "opencv2/core/core.hpp"

#include <QSharedPointer>

#include <map>

namespace Z3D
{

class ZStructuredLightPattern
{
public:
    explicit ZStructuredLightPattern();

    void updatePointCount();

    std::map<int, std::vector<cv::Vec2f> > fringePointsList;
    int estimatedCloudPoints;
};

} // namespace Z3D
