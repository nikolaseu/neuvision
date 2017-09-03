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

#include "zsinglecamerastereosls.h"

#include "zcameracalibrationprovider.h"
#include "zcalibratedcameraprovider.h"

namespace Z3D
{

ZSingleCameraStereoSLS::ZSingleCameraStereoSLS(QObject *parent)
    : ZStereoSLS(parent)
{

}

ZSingleCameraStereoSLS::~ZSingleCameraStereoSLS()
{

}

void ZSingleCameraStereoSLS::processPatterns()
{
    /// is there something to process?
    if (!projectedPattern || decodedPatterns.empty()) {
        return;
    }

    int estimatedCloudPoints = projectedPattern->estimatedCloudPoints;
    for (const auto &decodedPattern : decodedPatterns)
        estimatedCloudPoints += decodedPattern->estimatedCloudPoints;

    Z3D::ZSimplePointCloud::Ptr cloud = triangulate(
                decodedPatterns[0]->intensityImg,
                decodedPatterns[0]->fringePointsList,
            projectedPattern->fringePointsList,
            estimatedCloudPoints);

    if (cloud) {
        emit scanFinished(cloud);
    }

    projectedPattern = ZProjectedPattern::Ptr(nullptr);
    decodedPatterns.clear();
}

QString ZSingleCameraStereoSLS::id() const
{
    return QString("Projector+Camera");
}

QString ZSingleCameraStereoSLS::displayName() const
{
    return QString("Projector + Camera");
}

void ZSingleCameraStereoSLS::init(QSettings *settings)
{
    camera = CalibratedCameraProvider::getCalibratedCamera(settings);

    settings->beginGroup("ProjectorCalibration");
    {
        projectorCalibration =  ZCameraCalibrationProvider::getCalibration(settings);
    }
    settings->endGroup();

    setLeftCalibration(camera->calibration());
    setRightCalibration(projectorCalibration);

    std::vector<Z3D::ZCameraInterface::Ptr> camerasVector;
    camerasVector.push_back(camera->camera());
    setAcquisitionManager(new ZCameraAcquisitionManager(camerasVector));
}

void ZSingleCameraStereoSLS::onPatternProjected(ZProjectedPattern::Ptr pattern)
{
    if (pattern && pattern->estimatedCloudPoints > 0) {
        projectedPattern = pattern;
        processPatterns();
    } else {
        decodedPatterns.clear();
    }
}

void ZSingleCameraStereoSLS::onPatternsDecoded(std::vector<ZDecodedPattern::Ptr> patterns)
{
    for (auto decodedPattern : patterns) {
        if (decodedPattern->estimatedCloudPoints < 1) {
            projectedPattern = ZProjectedPattern::Ptr(nullptr);
            return;
        }
    }

    decodedPatterns = patterns;
    processPatterns();
}

} // namespace Z3D
