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

#include "zcameraimage.h"

#include "string.h" // memcpy

#include <QDebug>
#include <QMetaType>

//! FIXME esto no va acá!!
static int z3dImagePtrTypeId = qRegisterMetaType<Z3D::ZImageGrayscale::Ptr>("Z3D::ZImageGrayscale::Ptr");

namespace Z3D
{

ZImageGrayscale::ZImageGrayscale(int width, int height, int xOffset, int yOffset, int bytesPerPixel)
    : m_number(0)
    , m_width(width)
    , m_height(height)
    , m_xOffset(xOffset)
    , m_yOffset(yOffset)
    , m_bytesPerPixel(bytesPerPixel)
{
    switch (m_bytesPerPixel) {
    case 1:
        m_cvMat.create(m_height, m_width, CV_8UC1);
        break;
    case 2:
        m_cvMat.create(m_height, m_width, CV_16UC1);
        break;
    default:
        qCritical() << "invalid image, only 8 or 16 bits per pixel are supported";
        break;
    }

    //qDebug() << Q_FUNC_INFO << (long)this << bufferSize();
}

ZImageGrayscale::ZImageGrayscale(QString filename)
    : m_number(0)
    , m_xOffset(0)
    , m_yOffset(0)
{
    try {
        m_cvMat = cv::imread(qPrintable(filename), cv::IMREAD_GRAYSCALE);

        switch (m_cvMat.type()) {
        case CV_8UC1:
            m_bytesPerPixel = 1;
            break;
        case CV_16UC1:
            m_bytesPerPixel = 2;
            break;
        default:
            qCritical() << "invalid image, only 8 or 16 bits per pixel are supported";
            m_cvMat = cv::Mat();
            break;
        }

        m_width = m_cvMat.cols;
        m_height = m_cvMat.rows;

    } catch (...) {
        qCritical() << "invalid image" << filename;
    }

    //qDebug() << Q_FUNC_INFO << (long)this << bufferSize();
}

ZImageGrayscale::ZImageGrayscale(int width, int height, int xOffset, int yOffset, int bytesPerPixel, void *externalBuffer)
    : m_number(0)
    , m_width(width)
    , m_height(height)
    , m_xOffset(xOffset)
    , m_yOffset(yOffset)
    , m_bytesPerPixel(bytesPerPixel)
{
    switch (m_bytesPerPixel) {
    case 1:
        m_cvMat = cv::Mat(m_height, m_width, CV_8UC1, externalBuffer);
        break;
    case 2:
        m_cvMat = cv::Mat(m_height, m_width, CV_16UC1, externalBuffer);
        break;
    default:
        qCritical() << "invalid image, only 8 or 16 bits per pixel are supported";
        m_cvMat = cv::Mat();
        break;
    }

    //qDebug() << Q_FUNC_INFO << (long)this << bufferSize();
}

ZImageGrayscale::~ZImageGrayscale()
{
    //qDebug() << Q_FUNC_INFO << (long)this << bufferSize();
}

ZImageGrayscale::Ptr ZImageGrayscale::clone()
{
    ZImageGrayscale::Ptr clon(new ZImageGrayscale(width(), height(), xOffset(), yOffset(), bytesPerPixel()));

    clon->setBuffer( buffer() );
    clon->setNumber( number() );

    return clon;
}

unsigned char *ZImageGrayscale::buffer() const
{
    return m_cvMat.data;
}

bool ZImageGrayscale::setBuffer(void *otherBuffer)
{
    return 0 != memcpy(m_cvMat.data, otherBuffer, bufferSize());
}

bool ZImageGrayscale::save(QString fileName)
{
    return cv::imwrite(qPrintable(fileName), m_cvMat);
}

} // namespace Z3D
