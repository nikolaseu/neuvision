#pragma once

#include "opencv2/core/core.hpp"

#include "zstructuredlightpattern.h"

#include <QSharedPointer>

#include <map>

namespace Z3D
{

class ZDecodedPattern : public ZStructuredLightPattern
{
public:
    typedef QSharedPointer<ZDecodedPattern> Ptr;

    explicit ZDecodedPattern();

    cv::Mat intensityImg;
    cv::Mat maskImg;
    cv::Mat decodedImage;
    cv::Mat fringeImage;
};

} // namespace Z3D
