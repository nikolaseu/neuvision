#ifndef Z3D_STRUCTUREDLIGHT___ZBINARYPATTERNDECODER_H
#define Z3D_STRUCTUREDLIGHT___ZBINARYPATTERNDECODER_H

#include "opencv2/core/types.hpp"
#include <map>

namespace Z3D
{

namespace ZBinaryPatternDecoder
{

cv::Mat decodeBinaryPatternImages(const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &invImages, cv::Mat maskImg, bool isGrayCode = true);

cv::Mat simplifyBinaryPatternData(cv::Mat image, cv::Mat maskImg, std::map<int, std::vector<cv::Vec2f> > &fringePoints);

} // namespace ZBinaryPatternDecoder

}// namespace Z3D

#endif // Z3D_STRUCTUREDLIGHT___ZBINARYPATTERNDECODER_H
