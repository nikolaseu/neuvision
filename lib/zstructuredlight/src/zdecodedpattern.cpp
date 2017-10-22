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

#include "zdecodedpattern.h"

#include <QMetaType>

namespace Z3D
{

static int z3dDecodedPatternPtrTypeId = qRegisterMetaType<Z3D::ZDecodedPattern::Ptr>("Z3D::DecodedPattern::Ptr");

ZDecodedPattern::ZDecodedPattern(std::map<int, std::vector<cv::Vec2f> > fringePointsList,
                                 cv::Mat intensityImg,
                                 cv::Mat maskImg,
                                 cv::Mat decodedImage,
                                 cv::Mat fringeImage)
    : ZStructuredLightPattern(fringePointsList)
    , m_intensityImg(intensityImg)
    , m_maskImg(maskImg)
    , m_decodedImage(decodedImage)
    , m_fringeImage(fringeImage)
{

}

cv::Mat ZDecodedPattern::intensityImg() const
{
    return m_intensityImg;
}

cv::Mat ZDecodedPattern::maskImg() const
{
    return m_maskImg;
}

cv::Mat ZDecodedPattern::decodedImage() const
{
    return m_decodedImage;
}

cv::Mat ZDecodedPattern::fringeImage() const
{
    return m_fringeImage;
}

} // namespace Z3D
