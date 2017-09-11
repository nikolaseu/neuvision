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

#include "zstructuredlightsystem.h"
#include "zcameracalibration.h"

namespace Z3D
{

class ZStereoSystemImpl;

class ZStereoSLS : public ZStructuredLightSystem
{
    Q_OBJECT

    Q_PROPERTY(double maxValidDistance READ maxValidDistance WRITE setMaxValidDistance NOTIFY maxValidDistanceChanged)

public:
    explicit ZStereoSLS(QObject *parent = nullptr);
    ~ZStereoSLS();

    double maxValidDistance() const;

signals:
    void maxValidDistanceChanged(double maxValidDistance);

public slots:
    void setMaxValidDistance(double maxValidDistance);

protected slots:
    void initialize();

    void setLeftCalibration(Z3D::ZCameraCalibration::Ptr cameraCalibration);
    void setRightCalibration(Z3D::ZCameraCalibration::Ptr cameraCalibration);

    Z3D::ZSimplePointCloud::Ptr triangulate(const cv::Mat &intensityImg,
            const std::map<int, std::vector<cv::Vec2f> > &leftPoints,
            const std::map<int, std::vector<cv::Vec2f> > &rightPoints,
            int maxPosibleCloudPoints);

private:
    double m_maxValidDistance;
    ZStereoSystemImpl *m_stereoSystem;
};

} // namespace Z3D
