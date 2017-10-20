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

#include "zcirclegridcalibrationpatternfinder.h"

#include <opencv2/calib3d/calib3d.hpp>

namespace Z3D
{

ZCircleGridCalibrationPatternFinder::ZCircleGridCalibrationPatternFinder(QObject *parent)
    : ZCalibrationPatternFinder(parent)
{
    /// generate current config hash
    updateConfigHash();
}

QString ZCircleGridCalibrationPatternFinder::id() const
{
    return QString(metaObject()->className());
}

QString ZCircleGridCalibrationPatternFinder::name() const
{
    return QLatin1String("Circle grid");
}

bool ZCircleGridCalibrationPatternFinder::findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &patternPoints)
{
    bool found = cv::findCirclesGrid(image, m_boardSize, corners);

    if (found) {
        /// clear vector
        patternPoints.clear();

        /// reserve space
        patternPoints.reserve(columns() * rows());

        /// generate world object coordinates
        for (int h=0; h<rows(); ++h) {
            for (int w=0; w<columns(); ++w) {
                patternPoints.push_back(cv::Point3f(w, h, 0.f));
            }
        }
    }

    return found;
}

void ZCircleGridCalibrationPatternFinder::updateConfigHash()
{
    /// this doesn't have extra configuration, base hash already includes the type of the pattern
    QString hash = getBaseHash();

    if (m_configHash != hash) {
        m_configHash = hash;
        emit configHashChanged(m_configHash);
    }
}

} // namespace Z3D
