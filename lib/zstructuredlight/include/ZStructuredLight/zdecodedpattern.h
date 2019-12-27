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

#include "ZStructuredLight/zstructuredlight_global.h"

#include "ZStructuredLight/zstructuredlightpattern.h"

#include <opencv2/core/mat.hpp>

namespace Z3D
{

struct Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZDecodedPattern : public ZStructuredLightPattern
{
public:
    static constexpr float_t NO_VALUE = std::numeric_limits<float_t>::quiet_NaN();
    static inline bool isValidValue(float_t value) { return !std::isnan(value); }

    explicit ZDecodedPattern(const cv::Mat &decodedImage,
                             const cv::Mat &intensityImg);

    cv::Mat intensityImg() const;

private:
    cv::Mat m_intensityImg;
};

} // namespace Z3D
