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

#include "zdualcamerastereosls.h"

#include "zcamerainterface.h"
#include "zdecodedpattern.h"
#include "zsimplepointcloud.h"

#include <QDebug>
#include <QSettings>
#include <QTime>

namespace Z3D
{

ZDualCameraStereoSLS::ZDualCameraStereoSLS(ZCameraList cameras,
                                           ZMultiCameraCalibrationPtr stereoCalibration,
                                           ZPatternProjectionPtr patternProjection,
                                           QObject *parent)
    : ZStereoSLS(cameras, stereoCalibration, patternProjection, parent)
    , m_cameras(cameras)
{
    if (m_cameras.size() != 2) {
        qWarning() << "there must be exactly 2 cameras!";
        return;
    }
}

ZDualCameraStereoSLS::~ZDualCameraStereoSLS()
{

}

ZCameraList ZDualCameraStereoSLS::cameras() const
{
    return m_cameras;
}

void ZDualCameraStereoSLS::onPatternProjected(ZProjectedPatternPtr pattern)
{
    Q_UNUSED(pattern);
}

void ZDualCameraStereoSLS::onPatternsDecoded(std::vector<ZDecodedPatternPtr> decodedPatterns)
{
    QTime startTime;
    startTime.start();

    Z3D::ZSimplePointCloudPtr cloud = triangulate(decodedPatterns[0]->intensityImg(),
            decodedPatterns[0]->decodedImage(),
            decodedPatterns[1]->decodedImage());

    if (cloud) {
        qDebug() << "finished calculating point cloud with" << cloud->width() * cloud->height()
                 << "points in" << startTime.elapsed() << "msecs";
        emit scanFinished(cloud);
    } else {
        qDebug() << "finished calculating point cloud in" << startTime.elapsed() << "msecs";
    }
}

} // namespace Z3D
