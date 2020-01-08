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

    for (size_t k = 0; k < 2; k++) {
        cv::initUndistortRectifyMap(m_calibration->cameraMatrix[k], m_calibration->distCoeffs[k], m_R[k], m_P[k], m_imageSize, CV_16SC2, m_rectifyMaps[k][0], m_rectifyMaps[k][1]);
    }

    setReady(true);
}

template<typename T>
ZPointCloudPtr process(const cv::Mat &colorImg, const cv::Mat& Q, cv::Mat leftImg, cv::Mat rightImg)
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
            if (!ZDecodedPattern::isValidValue(*imgData)) {
                //qDebug() << "skipping left pixel x:" << x << "-> no data";
                continue;
            }

            //qDebug() << "processing col" << x << "value" << *imgData;

            bool shouldContinueInRight = true;
            for (; shouldContinueInRight && rx<imgWidth-1; ++rx, ++rImgData, ++rImgDataNext) {
                if (!ZDecodedPattern::isValidValue(*rImgData)) {
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
                if (*imgData >= *rImgData && *imgData < *rImgDataNext
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

    cv::Mat xyz[3];
    cv::split(pointCloud, xyz);
    cv::Mat &xChannel = xyz[0];
    cv::Mat &yChannel = xyz[1];
    cv::Mat &zChannel = xyz[2];

    /// Gradients to compute normals
    cv::Mat deltas[3][2];

    const cv::Mat_<float> diffKernelX = (cv::Mat_<float>(1, 3) << -1, 0, 1) / 2;
    cv::filter2D(xChannel, deltas[0][0], CV_32F, diffKernelX);
    cv::filter2D(yChannel, deltas[1][0], CV_32F, diffKernelX);
    cv::filter2D(zChannel, deltas[2][0], CV_32F, diffKernelX);

    const cv::Mat_<float> diffKernelY = (cv::Mat_<float>(3, 1) << -1, 0, 1) / 2;
    cv::filter2D(xChannel, deltas[0][1], CV_32F, diffKernelY);
    cv::filter2D(yChannel, deltas[1][1], CV_32F, diffKernelY);
    cv::filter2D(zChannel, deltas[2][1], CV_32F, diffKernelY);

    cv::Mat invalidDataMask = disparityImage == 0;
    cv::dilate(invalidDataMask, invalidDataMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    ZSimplePointCloud::PointVector points(pointCloud.size().area());
    std::vector<int> pointIndices(points.size(), -1); // where each point was added
    std::vector<uint32_t> faceIndices;
    faceIndices.reserve(3 * 2 * points.size()); // reserve approx maximum size

    size_t validPointIndex = 0;
    for (int y=0; y<imgHeight; ++y) {
        const uint8_t* invalidData = invalidDataMask.ptr<uint8_t>(y);
        const cv::Vec3f* positionData = pointCloud.ptr<cv::Vec3f>(y);
        const uint8_t* colorData = colorImg.ptr<uint8_t>(y);

        const float_t *dxdx = deltas[0][0].ptr<float_t>(y);
        const float_t *dydx = deltas[1][0].ptr<float_t>(y);
        const float_t *dzdx = deltas[2][0].ptr<float_t>(y);
        const float_t *dxdy = deltas[0][1].ptr<float_t>(y);
        const float_t *dydy = deltas[1][1].ptr<float_t>(y);
        const float_t *dzdy = deltas[2][1].ptr<float_t>(y);

        for (int x = 0; x < imgWidth; ++x, ++invalidData, ++positionData, ++colorData,
                 ++dxdx, ++dydx, ++dzdx, ++dxdy, ++dydy, ++dzdy)
        {
            const size_t pointOrgIndex = y*imgWidth+x;

            // add point
            if (!*invalidData) {
                ZSimplePointCloud::PointType &point = points[validPointIndex];

                const cv::Vec3f &position = *positionData;
                point[0] = position[0];
                point[1] = position[1];
                point[2] = position[2];

                const cv::Vec3f dx(*dxdx, *dydx, *dzdx);
                const cv::Vec3f dy(*dxdy, *dydy, *dzdy);
                const cv::Vec3f vnormal = dy.cross(dx);
                cv::Vec3f normal;
                cv::normalize(vnormal, normal);
                point[3] = normal[0];
                point[4] = normal[1];
                point[5] = normal[2];

                union RGBAColor { // just to simplify converting color data
                    uchar rgba[4];
                    float asFloat;
                } color;

                color.rgba[0] = *colorData; // we don't really have RGB :(
                color.rgba[1] = *colorData;
                color.rgba[2] = *colorData;
                color.rgba[3] = 255;

                point[6] = color.asFloat;

                // radii (it's not really radii since it's not a circle, but ...)
                point[7] = std::min(0.1f, std::sqrt(float(cv::norm(vnormal)/M_PI)));

                // remember point index so we know it when required to add faces later
                pointIndices[pointOrgIndex] = validPointIndex;

                // valid data index/counter
                ++validPointIndex;
            }

#if 1
            // add face/s
            if (y < 1 || x < 1) {
                // we cannot add anything, we need previous row and previous column data
                continue;
            }

            const int thisIndex = pointIndices[pointOrgIndex];

            const size_t topOrgIndex = pointOrgIndex - imgWidth;
            const int topIndex = pointIndices[topOrgIndex];

            const size_t topLeftOrgIndex = topOrgIndex - 1;
            const int topLeftIndex = pointIndices[topLeftOrgIndex];

            const size_t leftOrgIndex = pointOrgIndex - 1;
            const int leftIndex = pointIndices[leftOrgIndex];

            //! TODO make parameter, also probably using a maximum angle is easier to setup
            constexpr float zThreshold = 1.f;

            auto addThisAndTopLeftAndLeftTriangle = [&]() {
                const auto thisZ = points[thisIndex][2];
                const auto topLeftZ = points[topLeftIndex][2];
                const auto leftZ = points[leftIndex][2];
                if (std::abs(thisZ - leftZ) < zThreshold
                    && std::abs(topLeftZ - leftZ) < zThreshold)
                {
                    faceIndices.push_back(thisIndex);
                    faceIndices.push_back(topLeftIndex);
                    faceIndices.push_back(leftIndex);
                }
            };

            auto addThisAndTopAndTopLeftTriangle = [&]() {
                const auto thisZ = points[thisIndex][2];
                const auto topZ = points[topIndex][2];
                const auto topLeftZ = points[topLeftIndex][2];
                if (std::abs(thisZ - topZ) < zThreshold
                    && std::abs(topLeftZ - topZ) < zThreshold)
                {
                    faceIndices.push_back(thisIndex);
                    faceIndices.push_back(topIndex);
                    faceIndices.push_back(topLeftIndex);
                }
            };

            auto addThisAndTopAndLeftTriangle = [&]() {
                const auto thisZ = points[thisIndex][2];
                const auto topZ = points[topIndex][2];
                const auto leftZ = points[leftIndex][2];
                if (std::abs(topZ - thisZ) < zThreshold
                    && std::abs(leftZ - thisZ) < zThreshold)
                {
                    faceIndices.push_back(thisIndex);
                    faceIndices.push_back(topIndex);
                    faceIndices.push_back(leftIndex);
                }
            };

            auto addTopAndTopLeftAndLeftTriangle = [&]() {
                const auto topZ = points[topIndex][2];
                const auto topLeftZ = points[topLeftIndex][2];
                const auto leftZ = points[leftIndex][2];
                if (std::abs(topZ - topLeftZ) < zThreshold
                    && std::abs(leftZ - topLeftZ) < zThreshold)
                {
                    faceIndices.push_back(topIndex);
                    faceIndices.push_back(topLeftIndex);
                    faceIndices.push_back(leftIndex);
                }
            };

            if (thisIndex >= 0 && topLeftIndex >= 0 && leftIndex >= 0 && topIndex >= 0) {
                // we can to fill the complete quad, and we have two options.
                // to decide which one we'll use the difference of normals between the points on the
                // diagonals, so we join based on the surface direction change.
                // we could also chose based on the minimum diagonal distance, but we don't really care about
                // shape of triangles.
                auto getNormal = [](ZSimplePointCloud::PointType pointData) -> cv::Vec3f {
                    return cv::Vec3f(pointData[3], pointData[4], pointData[5]);
                };

                const cv::Vec3f normalTopLeft = getNormal(points[topLeftIndex]);
                const cv::Vec3f normalLeft = getNormal(points[leftIndex]);
                const cv::Vec3f normalTop = getNormal(points[topIndex]);
                const cv::Vec3f normal = getNormal(points[thisIndex]);
                const float cosAngleBetweenThisAndTopLeft = normalTopLeft.dot(normal);
                const float cosAngleBetweenLeftAndTop = normalLeft.dot(normalTop);
                if (cosAngleBetweenThisAndTopLeft > cosAngleBetweenLeftAndTop) {
                    // add -45 diagonal
                    addThisAndTopLeftAndLeftTriangle();
                    addThisAndTopAndTopLeftTriangle();
                } else {
                    // add +45 diagonal
                    addThisAndTopAndLeftTriangle();
                    addTopAndTopLeftAndLeftTriangle();
                }
            } else if (thisIndex >= 0 && topLeftIndex >= 0 && leftIndex >= 0) {
                // top is missing
                addThisAndTopLeftAndLeftTriangle();
            } else if (thisIndex >= 0 && topIndex >= 0 && topLeftIndex >= 0) {
                // left is missing
                addThisAndTopAndTopLeftTriangle();
            } else if (thisIndex >= 0 && topIndex >= 0 && leftIndex >= 0) {
                // top left is missing
                addThisAndTopAndLeftTriangle();
            } else if (topIndex >= 0 && topLeftIndex >= 0 && leftIndex >= 0) {
                // this is missing
                addTopAndTopLeftAndLeftTriangle();
            }
#endif
        }
    }

    if (validPointIndex < 1) {
        return nullptr;
    }

    points.resize(validPointIndex);

    return ZPointCloudPtr(new ZSimplePointCloud(points, faceIndices));
}

Z3D::ZPointCloudPtr ZStereoSystemImpl::triangulate(const cv::Mat &leftColorImage, const cv::Mat &leftDecodedImage, const cv::Mat &rightDecodedImage)
{
    cv::Mat leftColorRemapedImage;
    cv::remap(leftColorImage, leftColorRemapedImage, m_rectifyMaps[0][0], m_rectifyMaps[0][1], cv::INTER_LINEAR);
    cv::Mat leftRemapedImage;
    cv::remap(leftDecodedImage, leftRemapedImage, m_rectifyMaps[0][0], m_rectifyMaps[0][1], cv::INTER_LINEAR);
    cv::Mat rightRemapedImage;
    cv::remap(rightDecodedImage, rightRemapedImage, m_rectifyMaps[1][0], m_rectifyMaps[1][1], cv::INTER_LINEAR);

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
