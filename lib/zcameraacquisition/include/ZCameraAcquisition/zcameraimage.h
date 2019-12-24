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

#include "zcameraacquisition_fwd.h"
#include "zcameraacquisition_global.h"

#include <opencv2/core/mat.hpp>

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZImageGrayscale
{

public:
    /// create new image
    explicit ZImageGrayscale(int width, int height, int xOffset = 0, int yOffset = 0, int bytesPerPixel = 1);

    /// create new image from OpenCV cv::Mat
    explicit ZImageGrayscale(cv::Mat mat);

    /// create new image from externally managed buffer
    explicit ZImageGrayscale(int width, int height, int xOffset, int yOffset, int bytesPerPixel, void *externalBuffer);

    ~ZImageGrayscale();

    /// creates a complete copy of the instance
    ZCameraImagePtr clone();

    inline cv::Mat cvMat() const { return m_cvMat; }

    unsigned char *buffer() const;
    bool setBuffer(void *otherBuffer);

    inline long number() const { return m_number; }
    inline void setNumber(long number) { m_number = number; }

    inline int width() const { return m_width; }

    inline int height() const { return m_height; }

    inline int xOffset() const { return m_xOffset; }

    inline int yOffset() const { return m_yOffset;  }

    inline int bytesPerPixel() const { return m_bytesPerPixel; }

    inline int bufferSize() const { return m_width * m_height * m_bytesPerPixel; }

private:
    cv::Mat m_cvMat;
    const int m_width;
    const int m_height;
    const int m_xOffset;
    const int m_yOffset;
    const int m_bytesPerPixel;
    long m_number;
};

namespace ZCameraImage {
Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraImagePtr fromFile(const QString& fileName);
Z3D_CAMERAACQUISITION_SHARED_EXPORT bool save(const ZCameraImagePtr &image, const QString &fileName);
} // namespace ZCameraImage

} // namespace Z3D
