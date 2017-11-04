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

#include "zcameraacquisition_global.h"

#include <QSharedPointer>

#include "opencv2/opencv.hpp"

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZImageGrayscale
{

public:
    typedef std::shared_ptr<Z3D::ZImageGrayscale> Ptr;

    /// create new image
    ZImageGrayscale(int width, int height, int xOffset = 0, int yOffset = 0, int bytesPerPixel = 1);

    /// create new image from image file
    ZImageGrayscale(QString filename);

    /// create new image from externally managed  buffer
    ZImageGrayscale(int width, int height, int xOffset, int yOffset, int bytesPerPixel, void *externalBuffer);

    ~ZImageGrayscale();

    /// creates a complete copy of the instance
    ZImageGrayscale::Ptr clone();

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

    bool save(QString fileName);

private:
    long m_number;
    int m_width;
    int m_height;
    int m_xOffset;
    int m_yOffset;
    int m_bytesPerPixel;

    cv::Mat m_cvMat;
};

} // namespace Z3D
