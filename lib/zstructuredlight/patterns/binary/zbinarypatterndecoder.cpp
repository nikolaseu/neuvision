//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "zbinarypatterndecoder.h"

#include "zdecodedpattern.h"

#include <map>

#include <opencv2/imgproc.hpp>

#include <QDebug>

namespace Z3D
{

namespace ZBinaryPatternDecoder
{

constexpr const uint16_t NO_VALUE = std::numeric_limits<uint16_t>::quiet_NaN();

unsigned int binaryToGray(unsigned int num)
{
    return (num >> 1) ^ num;
}


unsigned int grayToBinary(unsigned int num)
{
    unsigned int len = 8 * sizeof(num);
    unsigned int binary = 0;

    /// mask the MSB.
    unsigned int grayBit = 1 << ( len - 1 );
    /// copy the MSB.
    binary = num & grayBit;
    /// store the bit we just set.
    unsigned int binBit = binary;
    /// traverse remaining Gray bits.
    while( grayBit >>= 1 ) {
        /// shift the current binary bit
        /// to align with the Gray bit.
        binBit >>= 1;
        /// XOR the two bits.
        binary |= binBit ^ ( num & grayBit );
        /// store the current binary bit
        binBit = binary & grayBit;
    }

    return binary;
}


cv::Mat decodeBinaryPatternImages(const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &invImages, cv::Mat maskImg, bool isGrayCode)
{
    const size_t imgCount = images.size();

    const cv::Size &imgSize = images[0].size();

    /// 16 bits is more than enough
    cv::Mat decodedImg(imgSize, CV_16UC1, cv::Scalar(NO_VALUE));

    const int &imgHeight = imgSize.height;
    const int &imgWidth = imgSize.width;

    for (size_t i=0; i<imgCount; ++i) {
        const cv::Mat &regImg = images[i];
        const cv::Mat &invImg = invImages[i];
        uint16_t bit = 1 << ( imgCount-i-1 );
        for (int y=0; y<imgHeight; ++y) {
            /// get pointers to first item of the row
            const uint8_t* maskImgData = maskImg.ptr<uint8_t>(y);
            const uint8_t* imgData = regImg.ptr<uint8_t>(y);
            const uint8_t* invImgData = invImg.ptr<uint8_t>(y);
            uint16_t* decodedImgData = decodedImg.ptr<uint16_t>(y);
            for (int x=0; x<imgWidth; ++x) {
                if (*maskImgData) {
                    uint16_t &value = *decodedImgData;
                    if (*imgData > *invImgData) {
                        /// enable bit
                        value |= bit;
                    }
                }

                /// don't forget to advance pointers!!
                maskImgData++;
                imgData++;
                invImgData++;
                decodedImgData++;
            }
        }
    }

    /// convert gray code to binary, i.e. "normal" value
    if (isGrayCode) {
        for (int y=0; y<imgHeight; ++y) {
            /// get pointers to first item of the row
            uint16_t* decodedImgData = decodedImg.ptr<uint16_t>(y);
            for (int x=0; x<imgWidth; ++x, ++decodedImgData) {
                uint16_t &value = *decodedImgData;
                if (value != NO_VALUE) {
                    value = grayToBinary(value);
//                    value = std::numeric_limits<uint16_t>::max() - grayToBinary(value); //! FIXME negative because projected patterns are going from high to low but should be fixed there!
                }
            }
        }
    }

    /// fill single pixel holes
    for (int y=0; y<imgHeight-1; ++y) {
        uint16_t *decodedImgData = decodedImg.ptr<uint16_t>(y) + 1; // start in x+1
        for (int x=1; x<imgWidth-1; ++x, ++decodedImgData) {
            uint16_t &value = *decodedImgData;
            if (value != NO_VALUE) {
                continue;
            }

            const int valueLeft = *(decodedImgData - 1);
            const int valueRight = *(decodedImgData + 1);
            if (valueLeft != NO_VALUE && valueRight == valueLeft) {
                /// assign value
                value = valueLeft;
            }
        }
    }

    /// to simplify shared code we use float for all decoded patterns
    cv::Mat decodedConvertedImg(imgSize, CV_32FC1, cv::Scalar(Z3D::ZDecodedPattern::NO_VALUE));
    decodedImg.convertTo(decodedConvertedImg, CV_32FC1);
    /// and also have to set empty pixels to the corresponding standard value
    decodedConvertedImg.setTo(Z3D::ZDecodedPattern::NO_VALUE, maskImg == 0);

    return decodedConvertedImg;
}


cv::Mat simplifyBinaryPatternData(cv::Mat image, cv::Mat maskImg, std::map<int, std::vector<cv::Vec2f> > &fringePoints)
{
    const cv::Size &imgSize = image.size();

    /// use 16 bits, it's enough
    cv::Mat decodedImg(imgSize, CV_16U, cv::Scalar(0));

    const int &imgHeight = imgSize.height;
    const int &imgWidth = imgSize.width;

    for (int y=0; y<imgHeight; ++y) {
        /// get pointers to first item of the row
        const uint8_t* maskImgData = maskImg.ptr<uint8_t>(y);
        const uint16_t* imgData = image.ptr<uint16_t>(y);
        uint16_t* decodedImgData = decodedImg.ptr<uint16_t>(y);
        for (int x=0; x<imgWidth-1; ++x) {
            /// get mask pixels
            const uint8_t &maskValue     = *maskImgData;
            maskImgData++; /// always advance pointer
            const uint8_t &nextMaskValue = *maskImgData;

            /// get image pixels
            const uint16_t &value     = *imgData;
            imgData++; /// always advance pointer
            const uint16_t &nextValue = *imgData;

            if (maskValue && nextMaskValue && nextValue != value) {
                *decodedImgData = value;
                //decodedImgData[x+1] = nextValue; // not both borders, it's useless. x => x+0.5
                uint16_t imin = std::min(value, nextValue),
                               imax = std::max(value, nextValue);
                cv::Vec2f point(0.5f+x, y); // x => x+0.5
                if (false) { /// THIS SHOULD BE ALWAYS FALSE
                    /// todos?
                    for (int i = imin; i < imax; ++i) { // not "<=" because we use the left border only
                        fringePoints[i].push_back(point);
                    }
                } else {
                    /// solo "bordes"
                    fringePoints[imin].push_back(point);
                    if (imax - imin > 1)
                        fringePoints[imax-1].push_back(point);
                }
            }

            /// always advance pointer
            decodedImgData++;
        }
    }

    return decodedImg;
}

} // namespace ZBinaryPatternDecoder

} // namespace Z3D
