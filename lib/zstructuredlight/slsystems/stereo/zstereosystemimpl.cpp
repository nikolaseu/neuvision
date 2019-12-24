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

#include "zstereosystemimpl.h"

#include "ZStructuredLight/zdecodedpattern.h"
#include "ZStructuredLight/zgeometryutils.h"
#include "ZCameraCalibration/zopencvstereocameracalibration.h"
#include "ZCameraCalibration/zpinholecameracalibration.h"
#include "ZStructuredLight/zsimplepointcloud.h"

#include <QAtomicInt>
#include <QDateTime>
#include <QDebug>
#include <QFuture>
#include <QtConcurrentMap>
#include <QtConcurrentRun>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace Z3D
{

ZStereoSystemImpl::ZStereoSystemImpl(ZMultiCameraCalibrationPtr stereoCalibration, QObject *parent)
    : QObject(parent)
    , m_calibration(std::dynamic_pointer_cast<ZOpenCVStereoCameraCalibration>(stereoCalibration))
    , m_ready(false)
{
    m_R.resize(2);
    m_P.resize(2);

    m_imageSize = m_calibration->imageSize[0];

    QtConcurrent::run(this, &ZStereoSystemImpl::stereoRectify, 0);
}

ZStereoSystemImpl::~ZStereoSystemImpl()
{
    qDebug() << Q_FUNC_INFO;
}

bool ZStereoSystemImpl::ready() const
{
    return m_ready;
}

void ZStereoSystemImpl::stereoRectify(double alpha)
{
    qDebug() << Q_FUNC_INFO;

    setReady(false);

    /// we need calibrated cameras!
    if (!m_calibration) {
        qWarning() << "invalid calibration! this only works for stereo calibration!";
        return;
    }

    cv::Rect validRoi[2];

    cv::stereoRectify(m_calibration->cameraMatrix[0], m_calibration->distCoeffs[0],
                      m_calibration->cameraMatrix[1], m_calibration->distCoeffs[1],
                      m_imageSize,
                      m_calibration->R, m_calibration->T,
                      m_R[0], m_R[1],
                      m_P[0], m_P[1],
                      m_Q,
                      /** flags – Operation flags that may be zero or CV_CALIB_ZERO_DISPARITY . If the flag is set, the function makes the principal points of each camera have the same pixel coordinates in the rectified views. And if the flag is not set, the function may still shift the images in the horizontal or vertical direction (depending on the orientation of epipolar lines) to maximize the useful image area. */
                      0 /*CALIB_ZERO_DISPARITY*/,
                      /** alpha – Free scaling parameter. If it is -1 or absent, the function performs the default scaling. Otherwise, the parameter should be between 0 and 1. alpha=0 means that the rectified images are zoomed and shifted so that only valid pixels are visible (no black areas after rectification). alpha=1 means that the rectified image is decimated and shifted so that all the pixels from the original images from the cameras are retained in the rectified images (no source image pixels are lost). Obviously, any intermediate value yields an intermediate result between those two extreme cases.*/
                      alpha,
                      m_imageSize, &validRoi[0], &validRoi[1]);

    setReady(true);
}

template<typename T>
ZPointCloudPtr process(const cv::Mat &colorImg, cv::Mat Q, cv::Mat leftImg, cv::Mat rightImg)
{
    const cv::Size &imgSize = leftImg.size();
    const int &imgHeight = imgSize.height;
    const int &imgWidth = imgSize.width;

    cv::Mat disparityImage(imgSize, CV_32FC1, cv::Scalar(0));

    for (int y=0; y<imgHeight; ++y) {
        //qDebug() << "processing row" << y;

        const uint8_t* colorData = colorImg.ptr<uint8_t>(y);
        float *depthPtr = disparityImage.ptr<float>(y);
        T* imgData = leftImg.ptr<T>(y);
        T* rImgData = rightImg.ptr<T>(y);
        T* rImgDataNext = rImgData + 1;
        for (int x=0, rx=0; x<imgWidth; ++x, ++imgData, ++colorData, ++depthPtr) {
            if (*imgData == ZDecodedPattern::NO_VALUE) {
                //qDebug() << "skipping left pixel x:" << x << "-> no data";
                continue;
            }

            //qDebug() << "processing col" << x << "value" << *imgData;

            bool shouldContinueInRight = true;
            for (; shouldContinueInRight && rx<imgWidth-1; ++rx, ++rImgData, ++rImgDataNext) {
                if (*rImgData == ZDecodedPattern::NO_VALUE) {
                    //qDebug() << "skipping right pixel x:" << x << "-> no data";
                    continue;
                }

                //qDebug() << "processing right col" << rx << "value" << *rImgData << "next" << *rImgDataNext;

                if (*imgData < *rImgData) {
                    //qDebug() << "value in right image is higher than left image, advancing left pointer...";
                    shouldContinueInRight = false;
                    break;
                }

                //! TODO this is a really naive way to find correspondeces ...
                if (*imgData >= *rImgData && *imgData <= *rImgDataNext
                        && (*rImgDataNext - *rImgData) < 2.f)
                {
                    const float range = *rImgDataNext - *rImgData;
                    const float offset = range > FLT_EPSILON
                            ? (*imgData - *rImgData) / range
                            : 0;
                    *depthPtr = float(x) - (float(rx) + offset);

                    shouldContinueInRight = false;
                    break;
                }
            }
        }
    }

    //! Reproject to compute xyz
    cv::Mat pointCloud(imgSize, CV_32FC3);
    cv::reprojectImageTo3D(disparityImage, pointCloud, Q);

    cv::Mat zChannel;
    cv::extractChannel(pointCloud, zChannel, 2);

    /// Gradients of z to compute normals
    cv::Mat gradientX, gradientY;
    const int sobelScale = 1;
    const int sobelDelta = 0;
    const int sobelDataTypeDepth = CV_32F;
    cv::Sobel(zChannel, gradientX, sobelDataTypeDepth, 1, 0, 3, sobelScale, sobelDelta, cv::BORDER_DEFAULT );
    cv::Sobel(zChannel, gradientY, sobelDataTypeDepth, 0, 1, 3, sobelScale, sobelDelta, cv::BORDER_DEFAULT );

    cv::Mat normalsMagnitude = gradientX.mul(gradientX) + gradientY.mul(gradientY) + 1;
    cv::sqrt(normalsMagnitude, normalsMagnitude);

    /// Normals
    cv::Mat normalsZ = 1 / normalsMagnitude;
    cv::Mat normalsX = gradientX.mul(normalsZ);
    cv::Mat normalsY = gradientY.mul(normalsZ);

    cv::Mat invalidDataMask = disparityImage == 0;
    cv::dilate(invalidDataMask, invalidDataMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    ZSimplePointCloud::PointVector points(pointCloud.size().area());

    size_t i=0;
    for (int y=0; y<imgHeight; ++y) {
        const uint8_t* invalidData = invalidDataMask.ptr<uint8_t>(y);
        const cv::Vec3f* positionData = pointCloud.ptr<cv::Vec3f>(y);
        const float_t *nxData = normalsX.ptr<float_t>(y);
        const float_t *nyData = normalsY.ptr<float_t>(y);
        const float_t *nzData = normalsZ.ptr<float_t>(y);
        const uint8_t* colorData = colorImg.ptr<uint8_t>(y);
        for (int x=0; x<imgWidth; ++x, ++invalidData, ++positionData, ++nxData, ++nyData, ++nzData, ++colorData) {
            if (*invalidData) {
                continue;
            }

            const cv::Vec3f &position = *positionData;
            points[i][0] = position[0];
            points[i][1] = position[1];
            points[i][2] = position[2];

            points[i][3] = *nxData;
            points[i][4] = *nyData;
            points[i][5] = - *nzData;

            // we don't really have RGB :(
            const uint32_t grayIntensity = (static_cast<uint32_t>(       255) << 24 | // alpha
                                            static_cast<uint32_t>(*colorData) << 16 | // b
                                            static_cast<uint32_t>(*colorData) <<  8 | // g
                                            static_cast<uint32_t>(*colorData));       // r
            points[i][6] = *reinterpret_cast<const float_t*>(&grayIntensity);

            // valid data index/counter
            ++i;
        }
    }
    points.resize(i);

    qDebug() << "found" << points.size() << "matches";
    if (points.size() < 1) {
        return nullptr;
    }

    return ZPointCloudPtr(new ZSimplePointCloud(points));
}

Z3D::ZPointCloudPtr ZStereoSystemImpl::triangulate(const cv::Mat &leftColorImage, const cv::Mat &leftDecodedImage, const cv::Mat &rightDecodedImage)
{
    //! TODO compute this once and keep in memory?
    cv::Mat rmap[2][2];
    for (size_t k = 0; k < 2; k++) {
        cv::initUndistortRectifyMap(m_calibration->cameraMatrix[k], m_calibration->distCoeffs[k], m_R[k], m_P[k], m_imageSize, CV_16SC2, rmap[k][0], rmap[k][1]);
    }

    cv::Mat leftColorRemapedImage;
    cv::remap(leftColorImage, leftColorRemapedImage, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
    cv::Mat leftRemapedImage;
    cv::remap(leftDecodedImage, leftRemapedImage, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
    cv::Mat rightRemapedImage;
    cv::remap(rightDecodedImage, rightRemapedImage, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);

    switch (leftRemapedImage.type()) {
    case CV_32FC1: // float_t
        return process<float_t>(leftColorRemapedImage, m_Q, leftRemapedImage, rightRemapedImage);
    default:
        qWarning() << "unkwnown image type:" << leftRemapedImage.type();
    }

    return nullptr;
}

void ZStereoSystemImpl::setReady(bool arg)
{
    if (m_ready == arg) {
        return;
    }

    m_ready = arg;
    emit readyChanged(arg);
}

} // namespace Z3D
