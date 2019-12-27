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

#include "ZStructuredLight/zcameraacquisitionmanager.h"

#include <QDebug>
#include <memory>

namespace Z3D
{

ZStereoSLS::ZStereoSLS(const ZCameraList &cameras,
                       const ZMultiCameraCalibrationPtr &stereoCalibration,
                       const ZPatternProjectionPtr &patternProjection,
                       QObject *parent)
    : ZStructuredLightSystem(std::make_shared<ZCameraAcquisitionManager>(cameras),
                             patternProjection,
                             parent)
    , m_maxValidDistance(0.001)
    , m_stereoSystem(new ZStereoSystemImpl(stereoCalibration))
{
    connect(m_stereoSystem, &ZStereoSystemImpl::readyChanged,
            this, &ZStereoSLS::setReady);
}

ZStereoSLS::~ZStereoSLS()
{
    qDebug() << "deleting stereo system...";
    delete m_stereoSystem;
}

double ZStereoSLS::maxValidDistance() const
{
    return m_maxValidDistance;
}

bool ZStereoSLS::setMaxValidDistance(double maxValidDistance)
{
    if (std::fabs(m_maxValidDistance - maxValidDistance) < DBL_EPSILON) {
        return true;
    }

    m_maxValidDistance = maxValidDistance;
    emit maxValidDistanceChanged(maxValidDistance);

    return true;
}

ZPointCloudPtr ZStereoSLS::triangulate(const cv::Mat &colorImg,
                                       const cv::Mat &leftDecodedImage,
                                       const cv::Mat &rightDecodedImage)
{
    return m_stereoSystem->triangulate(colorImg, leftDecodedImage, rightDecodedImage);
}

} // namespace Z3D
