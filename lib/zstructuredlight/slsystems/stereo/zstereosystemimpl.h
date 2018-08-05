/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "zstructuredlight_fwd.h"

#include "zpointcloud_fwd.h"
#include <Z3DCameraCalibration>

#include <QObject>

#include <opencv2/core/mat.hpp>

namespace Z3D
{

class ZStereoSystemImpl : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool ready READ ready WRITE setReady NOTIFY readyChanged)

public:
    explicit ZStereoSystemImpl(ZMultiCameraCalibrationPtr stereoCalibration, QObject *parent = nullptr);
    ~ZStereoSystemImpl();

    bool ready() const;

signals:
    void readyChanged(bool arg);

public slots:
    Z3D::ZPointCloudPtr triangulate(const cv::Mat &leftColorImage,
                                    const cv::Mat &leftDecodedImage,
                                    const cv::Mat &rightDecodedImage);

protected slots:
    void stereoRectify(double alpha = -1);
    void setReady(bool arg);

protected:
    ZStereoCameraCalibrationPtr m_calibration;
    cv::Size m_imageSize;
    std::vector<cv::Mat> m_R;
    std::vector<cv::Mat> m_P;
    cv::Mat m_Q;

private:
    bool m_ready;
};

} // namespace Z3D
