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

#include <map>

#include <opencv2/imgproc/imgproc.hpp>

#include <QDebug>

namespace Z3D
{

namespace ZBinaryPatternDecoder
{

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
    unsigned int imgCount = images.size();

    const cv::Size &imgSize = images[0].size();

    /// use 16 bits, it's more than enough
    cv::Mat decodedImg(imgSize, CV_16U, cv::Scalar(0));

    const int &imgHeight = imgSize.height;
    const int &imgWidth = imgSize.width;

    for (unsigned int i=0; i<imgCount; ++i) {
        cv::Mat regImg, invImg;

        if (false) {
            /// apply a median filter of 3x3
            cv::medianBlur(images[i], regImg, 3);
            cv::medianBlur(invImages[i], invImg, 3);
        } else {
            regImg = images[i];
            invImg = invImages[i];
        }

        //unsigned short bit = 1 << ( imgCount - i );
        unsigned short bit = 1 << ( i+1 );
        for (int y=0; y<imgHeight; ++y) {
            /// get pointers to first item of the row
            const unsigned char* maskImgData = maskImg.ptr<unsigned char>(y);
            const unsigned char* imgData = regImg.ptr<unsigned char>(y);
            const unsigned char* invImgData = invImg.ptr<unsigned char>(y);
            unsigned short* decodedImgData = decodedImg.ptr<unsigned short>(y);
            for (int x=0; x<imgWidth; ++x) {
                if (*maskImgData) {
                    unsigned short &value = *decodedImgData;
                    if (*imgData > *invImgData) {
                        /// enable bit
                        value |= bit;
                    } /*else {
                        /// disable bit
                        /// not neccessary, it starts with all bits in zero
                        value &= ~bit;
                    }*/
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
            const unsigned char* maskImgData = maskImg.ptr<unsigned char>(y);
            unsigned short* decodedImgData = decodedImg.ptr<unsigned short>(y);
            for (int x=0; x<imgWidth; ++x) {
                if (*maskImgData) {
                    unsigned short &value = *decodedImgData;
                    value = grayToBinary(value);

                    /// debug
                    //if (value % 2)
                    //    value += 64;
                }

                /// don't forget to advance pointers!!
                maskImgData++;
                decodedImgData++;
            }
        }
    }

    return decodedImg;
}


cv::Mat simplifyBinaryPatternData(cv::Mat image, cv::Mat maskImg, std::map<int, std::vector<cv::Vec2f> > &fringePoints)
{
    if (false) {
        /// apply a median filter of 3x3
        //cv::medianBlur(image, image, 3);

        /// values near irrelevant pixels should not be changed
        cv::Mat erodedMask(maskImg.size(), maskImg.type());
        //maskImg = maskImg > 0;
        //maskImg = maskImg * 255;
        cv::erode(maskImg, erodedMask, cv::Mat());
        cv::Mat bluredImage;
        cv::blur(image, bluredImage, cv::Size(3, 3));
        cv::Mat bluredMinusImage = bluredImage - image;

        cv::Mat erodedMask2(image.size(), image.type());
        erodedMask.convertTo(erodedMask2, image.type());
        //erodedMask2 = erodedMask2 * ((2^16)-1);

        /*qDebug() << "image" << image.type() << "size" << image.size().width << "x" << image.size().height << "\n"
                 << "bluredImage" << bluredImage.type() << "size" << bluredImage.size().width << "x" << bluredImage.size().height << "\n"
                 << "bluredMinusImage" << bluredMinusImage.type() << "size" << bluredMinusImage.size().width << "x" << bluredMinusImage.size().height << "\n"
                 << "erodedMask" << erodedMask.type() << "size" << erodedMask.size().width << "x" << erodedMask.size().height << "\n"
                 ;*/

        image = image + (bluredMinusImage & erodedMask2);
    }

    const cv::Size &imgSize = image.size();

    /// use 16 bits, it's enough
    cv::Mat decodedImg(imgSize, CV_16U, cv::Scalar(0));

    const int &imgHeight = imgSize.height;
    const int &imgWidth = imgSize.width;

    for (int y=0; y<imgHeight; ++y) {
        /// get pointers to first item of the row
        const unsigned char* maskImgData = maskImg.ptr<unsigned char>(y);
        const unsigned short* imgData = image.ptr<unsigned short>(y);
        unsigned short* decodedImgData = decodedImg.ptr<unsigned short>(y);
        for (int x=0; x<imgWidth-1; ++x) {
            /// get mask pixels
            const unsigned char &maskValue     = *maskImgData;
            maskImgData++; /// always advance pointer
            const unsigned char &nextMaskValue = *maskImgData;

            /// get image pixels
            const unsigned short &value     = *imgData;
            imgData++; /// always advance pointer
            const unsigned short &nextValue = *imgData;

            if (maskValue && nextMaskValue && nextValue != value) {
                *decodedImgData = value;
                //decodedImgData[x+1] = nextValue; // not both borders, it's useless. x => x+0.5
                unsigned short imin = std::min(value, nextValue),
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

    /// smooth
    for (auto it = fringePoints.begin(), itEnd = fringePoints.end(); it != itEnd; ++it) {
        if (false) {
            /// TEST: this is not what we want but maybe useful to smooth the lines?
            const double approxEps = 1.0;
            const std::vector<cv::Vec2f> &fringePointsVectorOrig = it->second;
            std::vector<cv::Vec2f> fringePointsVector;
            auto origSize = fringePointsVectorOrig.size();
            cv::approxPolyDP(fringePointsVectorOrig, fringePointsVector, approxEps, false);

            if (origSize != fringePointsVector.size()) {
                qDebug() << "reduced fringe points from" << origSize << "to" << fringePointsVector.size();
                it->second = fringePointsVector;
            }
        }

        if (false) {
            /// TEST
            auto &fringePointsVectorOrig = it->second;
            if (fringePointsVectorOrig.size() > 2) {
                //int counter = 0;
                auto leftIt = fringePointsVectorOrig.begin();
                auto endIt = fringePointsVectorOrig.end();
                auto midIt = leftIt + 1;
                auto rightIt = midIt + 1;
                cv::Vec2f leftPoint = *leftIt;
                cv::Vec2f midPoint;

                while (rightIt != endIt) {
                    midPoint = *midIt;

                    /// get (norm L2) ^ 2
                    /// don't use norm_L2 because its useless to calculate the sqrt
                    float sqrNorm = cv::norm(leftPoint - *rightIt, cv::NORM_L2SQR);
                    /// if it is <= 4 it means they are on the same line
                    /// if it is more than 10 they are too far away, don't use
                    if ( sqrNorm > 4. && sqrNorm < 10. ) {
                        *midIt = 0.5f * (leftPoint + *rightIt);
                        //counter++;
                    }

                    leftPoint = midPoint;

                    /// advance iterators
                    leftIt = midIt;
                    midIt = rightIt;
                    rightIt++;
                }

                //qDebug() << "modifyed" << counter << "of" << fringePointsVectorOrig.size();
            }

        }

    }

    return decodedImg;

    /*
    // Reduce noise with a kernel 3x3
    //cv::Mat denoised;
    //cv::blur( image, denoised, cv::Size(3,3) );

    cv::Mat dx(image.rows, image.cols, image.type());

    //void Sobel(InputArray src, OutputArray dst, int ddepth, int dx, int dy, int ksize=3, double scale=1, double delta=0, int borderType=BORDER_DEFAULT )¶
    cv::Sobel(image, dx, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_REPLICATE);
    return dx;


    /*cv::Mat detectedEdges;

    int lowThreshold;
    int ratio = 3;
    int kernel_size = 3;

    // Reduce noise with a kernel 3x3
    cv::blur( image, detectedEdges, cv::Size(3,3) );

    // Canny detector
    cv::Canny( detectedEdges, detectedEdges, lowThreshold, lowThreshold*ratio, kernel_size );

    // Using Canny's output as a mask, we display our result
    cv::Mat dst;
    dst = cv::Scalar::all(0);

    image.copyTo( dst, detectedEdges);

    return dst;*/
}

} // namespace ZBinaryPatternDecoder

} // namespace Z3D
