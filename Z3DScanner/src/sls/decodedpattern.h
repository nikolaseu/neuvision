#ifndef DECODEDPATTERN_H
#define DECODEDPATTERN_H

#include "opencv2/core/core.hpp"

#include <QSharedPointer>

#include <map>

namespace Z3D
{

class DecodedPattern
{
public:
    typedef QSharedPointer<DecodedPattern> Ptr;

    explicit DecodedPattern(int numOfCameras);

    std::vector< std::map<int, std::vector<cv::Vec2f> > > fringePointsList;
    cv::Mat intensityImg;
    std::vector<cv::Mat> maskImg;
    std::vector<cv::Mat> decodedImage;
    std::vector<cv::Mat> fringeImage;
    int estimatedCloudPoints;
    QString scanTmpFolder;
};

} // namespace Z3D

#endif // DECODEDPATTERN_H
