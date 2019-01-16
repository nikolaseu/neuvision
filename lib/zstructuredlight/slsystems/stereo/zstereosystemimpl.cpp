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

#include "zdecodedpattern.h"
#include "zgeometryutils.h"
#include "zpinhole/zopencvstereocameracalibration.h"
#include "zpinhole/zpinholecameracalibration.h"
#include "zsimplepointcloud.h"

#include <QAtomicInt>
#include <QDateTime>
#include <QDebug>
#include <QFuture>
#include <QtConcurrentMap>
#include <QtConcurrentRun>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

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
ZPointCloudPtr process(const cv::Mat &colorImg, cv::Mat Q, cv::Mat leftImg, cv::Mat rightImg) {
    const cv::Size &imgSize = leftImg.size();
    const int &imgHeight = imgSize.height;
    const int &imgWidth = imgSize.width;

    std::vector<cv::Vec3f> disparity;
    disparity.reserve(imgHeight * imgWidth); /// reserve maximum possible size
    std::vector<uint32_t> color;

    for (int y=0; y<imgHeight; ++y) {
        const uint8_t* colorData = colorImg.ptr<uint8_t>(y);
        const T* imgData = leftImg.ptr<T>(y);
        const T* rImgData = rightImg.ptr<T>(y);
        const T* rImgDataNext = rImgData + 1;
        for (int x=0, rx=0; x<imgWidth; ++x, ++imgData, ++colorData) {
            if (*imgData == ZDecodedPattern::NO_VALUE) {
                continue;
            }

            for (; rx<imgWidth-1; ++rx, ++rImgData, ++rImgDataNext) {
                if (*rImgData == ZDecodedPattern::NO_VALUE) {
                    continue;
                }

                if (*imgData < *rImgData) {
                    break;
                }

                //! TODO this is a really naive way to find correspondeces ...
                if (*imgData >= *rImgData && *imgData <= *rImgDataNext
                        && (*imgData - *rImgData) < 2.
                        && (*rImgDataNext - *rImgData) < 2.)
                {
                    const float range = *rImgDataNext - *rImgData;
                    const float offset = (*imgData - *rImgData) / range;
                    disparity.push_back(cv::Vec3f(x, y, float(x) - (float(rx) + offset)));

                    const uint32_t rgbWhite = (static_cast<uint32_t>(*colorData) << 24 | // alpha
                                               static_cast<uint32_t>(*colorData) << 16 | // r
                                               static_cast<uint32_t>(*colorData) <<  8 | // g
                                               static_cast<uint32_t>(*colorData));       // b
                    color.push_back(rgbWhite);
                    break;
                }
            }
        }
    }

    qDebug() << "found" << disparity.size() << "matches";

    std::vector<cv::Vec3f> points3f;
    cv::perspectiveTransform(disparity, points3f, Q);

    ZSimplePointCloud::PointVector points;
    points.resize(points3f.size());

    for (size_t i=0; i<points.size(); ++i) {
        points[i][0] = points3f[i][0];
        points[i][1] = points3f[i][1];
        points[i][2] = points3f[i][2];

        points[i][3] = *reinterpret_cast<const float_t*>(&color[i]);
    }

    return ZPointCloudPtr(new ZSimplePointCloud(points));
}

Z3D::ZPointCloudPtr ZStereoSystemImpl::triangulate(const cv::Mat &leftColorImage, const cv::Mat &leftDecodedImage, const cv::Mat &rightDecodedImage)
{
    //! TODO compute this once and keep in memory?
    cv::Mat rmap[2][2];
    for (int k = 0; k < 2; k++) {
        cv::initUndistortRectifyMap(m_calibration->cameraMatrix[k], m_calibration->distCoeffs[k], m_R[k], m_P[k], m_imageSize, CV_16SC2, rmap[k][0], rmap[k][1]);
    }

    cv::Mat leftColorRemapedImage;
    cv::remap(leftColorImage, leftColorRemapedImage, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
    cv::Mat leftRemapedImage;
    cv::remap(leftDecodedImage, leftRemapedImage, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
    cv::Mat rightRemapedImage;
    cv::remap(rightDecodedImage, rightRemapedImage, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);

    switch (leftRemapedImage.type()) {
    case CV_32F: // float
        return process<float>(leftColorRemapedImage, m_Q, leftRemapedImage, rightRemapedImage);
        break;
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
