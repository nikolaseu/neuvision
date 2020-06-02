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

#include "ZCore/zlogging.h"
#include "ZStructuredLight/zdecodedpattern.h"
#include "ZStructuredLight/zprojectedpattern.h"

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zstructuredlight.slsystems.stereo", QtInfoMsg)

namespace Z3D
{

ZSingleCameraStereoSLS::ZSingleCameraStereoSLS(const ZCameraPtr &camera,
                                               const ZMultiCameraCalibrationPtr &stereoCalibration,
                                               const ZPatternProjectionPtr &patternProjection,
                                               QObject *parent)
    : ZStereoSLS({camera}, stereoCalibration, patternProjection, parent)
{

}

ZSingleCameraStereoSLS::~ZSingleCameraStereoSLS()
{

}

const std::vector<ZSettingsItemPtr> &ZSingleCameraStereoSLS::settings()
{
    zDebug() << "returning settings for" << this;
    return m_settings;
}

void ZSingleCameraStereoSLS::processPatterns()
{
    /// is there something to process?
    if (!m_projectedPattern || m_decodedPatterns.empty()) {
        return;
    }

    Z3D::ZPointCloudPtr cloud = triangulate(m_decodedPatterns[0]->intensityImg(),
            m_decodedPatterns[0]->decodedImage(),
            m_projectedPattern->decodedImage());

    if (cloud) {
        emit scanFinished(cloud);
    }

    m_projectedPattern = nullptr;
    m_decodedPatterns.clear();
}

void ZSingleCameraStereoSLS::onPatternProjected(const ZProjectedPatternPtr &pattern)
{
    if (pattern) {
        m_projectedPattern = pattern;
        processPatterns();
    } else {
        m_decodedPatterns.clear();
    }
}

void ZSingleCameraStereoSLS::onPatternsDecoded(const std::vector<ZDecodedPatternPtr> &patterns)
{
    for (const auto &decodedPattern : patterns) {
        if (!decodedPattern) {
            m_projectedPattern = nullptr;
            return;
        }
    }

    m_decodedPatterns = patterns;
    processPatterns();
}

} // namespace Z3D
