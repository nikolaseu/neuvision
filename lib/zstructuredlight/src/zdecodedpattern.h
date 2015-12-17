#ifndef Z3D_STRUCTUREDLIGHT___ZDECODEDPATTERN_H
#define Z3D_STRUCTUREDLIGHT___ZDECODEDPATTERN_H

#include "opencv2/core/core.hpp"

#include <QSharedPointer>

#include <map>

namespace Z3D
{

class ZDecodedPattern
{
public:
    typedef QSharedPointer<ZDecodedPattern> Ptr;

    explicit ZDecodedPattern();

    void updatePointCount();

    std::map<int, std::vector<cv::Vec2f> > fringePointsList;
    cv::Mat intensityImg;
    cv::Mat maskImg;
    cv::Mat decodedImage;
    cv::Mat fringeImage;
    int estimatedCloudPoints;
};

} // namespace Z3D

#endif // Z3D_STRUCTUREDLIGHT___ZDECODEDPATTERN_H
