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

#include "zstructuredlightpattern.h"

namespace Z3D
{

ZStructuredLightPattern::ZStructuredLightPattern(std::map<int, std::vector<cv::Vec2f> > fringePointsList)
    : m_estimatedCloudPoints(0)
{
    m_fringePointsList = std::move(fringePointsList);

    for (auto it = fringePointsList.cbegin(), itEnd = fringePointsList.cend(); it != itEnd; ++it) {
        m_estimatedCloudPoints += it->second.size();
    }
}

std::map<int, std::vector<cv::Vec2f> > ZStructuredLightPattern::fringePointsList() const
{
    return m_fringePointsList;
}

int ZStructuredLightPattern::estimatedCloudPoints() const
{
    return m_estimatedCloudPoints;
}

} // namespace Z3D
