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

#include "ZStructuredLight/zstructuredlightsystem.h"

#include <Z3DCameraAcquisition>
#include <Z3DCameraCalibration>

#include <opencv2/core/mat.hpp>

namespace Z3D
{

class ZStereoSystemImpl;

class ZStereoSLS : public ZStructuredLightSystem
{
    Q_OBJECT

    Q_PROPERTY(double maxValidDistance READ maxValidDistance WRITE setMaxValidDistance NOTIFY maxValidDistanceChanged)

public:
    explicit ZStereoSLS(ZCameraList cameras,
                        ZMultiCameraCalibrationPtr stereoCalibration,
                        ZPatternProjectionPtr patternProjection,
                        QObject *parent = nullptr);

    ~ZStereoSLS() override;

    double maxValidDistance() const;

signals:
    void maxValidDistanceChanged(double maxValidDistance);

public slots:
    bool setMaxValidDistance(double maxValidDistance);

protected slots:
    Z3D::ZPointCloudPtr triangulate(const cv::Mat &colorImg,
                                    const cv::Mat &leftDecodedImage,
                                    const cv::Mat &rightDecodedImage);

private:
    double m_maxValidDistance;
    ZStereoSystemImpl *m_stereoSystem;
};

} // namespace Z3D
