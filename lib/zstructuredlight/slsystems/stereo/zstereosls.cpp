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

#include "zstereosls.h"

#include "zstereosystemimpl.h"

namespace Z3D
{

ZStereoSLS::ZStereoSLS(QObject *parent)
    : ZStructuredLightSystem(parent)
    , m_maxValidDistance(0.001)
    , m_stereoSystem(nullptr)
{
    /// finish initialization
    initialize();
}

ZStereoSLS::~ZStereoSLS()
{
    qDebug() << "deleting stereo system...";
    if (m_stereoSystem) {
        delete m_stereoSystem;
    }
}

double ZStereoSLS::maxValidDistance() const
{
    return m_maxValidDistance;
}

void ZStereoSLS::setMaxValidDistance(double maxValidDistance)
{
    if (std::fabs(m_maxValidDistance - maxValidDistance) < DBL_EPSILON) {
        return;
    }

    m_maxValidDistance = maxValidDistance;
    emit maxValidDistanceChanged(maxValidDistance);
}

void ZStereoSLS::initialize()
{
    m_stereoSystem = new ZStereoSystemImpl();

    connect(m_stereoSystem, &ZStereoSystemImpl::readyChanged,
            this, &ZStereoSLS::setReady);
}

void ZStereoSLS::setLeftCalibration(Z3D::ZCameraCalibration::Ptr cameraCalibration)
{
    m_stereoSystem->setLeftCameraCalibration(cameraCalibration);
}

void ZStereoSLS::setRightCalibration(Z3D::ZCameraCalibration::Ptr cameraCalibration)
{
    m_stereoSystem->setRightCameraCalibration(cameraCalibration);
}

ZSimplePointCloud::Ptr ZStereoSLS::triangulate(const cv::Mat &intensityImg,
                                               const std::map<int, std::vector<cv::Vec2f> > &leftPoints,
                                               const std::map<int, std::vector<cv::Vec2f> > &rightPoints,
                                               int maxPosibleCloudPoints)
{
    if (!m_stereoSystem) {
        return Z3D::ZSimplePointCloud::Ptr(nullptr);
    }

    return m_stereoSystem->triangulateOptimized(intensityImg, leftPoints, rightPoints, maxPosibleCloudPoints, m_maxValidDistance);
}

} // namespace Z3D
