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

#include "zdecodedpattern.h"
#include "zprojectedpattern.h"

namespace Z3D
{

ZSingleCameraStereoSLS::ZSingleCameraStereoSLS(
        ZCameraPtr camera,
        ZMultiCameraCalibrationPtr stereoCalibration,
        ZPatternProjectionPtr patternProjection,
        QObject *parent)
    : ZStereoSLS({ camera }, stereoCalibration, patternProjection, parent)
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

    Z3D::ZSimplePointCloudPtr cloud = triangulate(decodedPatterns[0]->intensityImg(),
            decodedPatterns[0]->decodedImage(),
            projectedPattern->decodedImage());

    if (cloud) {
        emit scanFinished(cloud);
    }

    projectedPattern = nullptr;
    decodedPatterns.clear();
}

void ZSingleCameraStereoSLS::onPatternProjected(ZProjectedPatternPtr pattern)
{
    if (pattern && pattern->estimatedCloudPoints() > 0) {
        projectedPattern = pattern;
        processPatterns();
    } else {
        decodedPatterns.clear();
    }
}

void ZSingleCameraStereoSLS::onPatternsDecoded(std::vector<ZDecodedPatternPtr> patterns)
{
    for (const auto &decodedPattern : patterns) {
        if (decodedPattern->estimatedCloudPoints() < 1) {
            projectedPattern = nullptr;
            return;
        }
    }

    decodedPatterns = patterns;
    processPatterns();
}

} // namespace Z3D
