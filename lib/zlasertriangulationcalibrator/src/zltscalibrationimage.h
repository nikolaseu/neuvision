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

#include "zcalibrationimage.h"

#include <QMetaType> // for Q_DECLARE_METATYPE, to use in QVariant

namespace Z3D
{

class ZLTSCalibrationImage : public ZCalibrationImage
{
    Q_OBJECT

public:
    typedef QSharedPointer<ZLTSCalibrationImage> Ptr;
    typedef QPointer<ZLTSCalibrationImage> WeakPtr;

    explicit ZLTSCalibrationImage(QString fileName, QObject *parent = 0);
    ~ZLTSCalibrationImage();

    virtual bool isValid();

    virtual QImage imageFromUrl(int viewStyle) const;
    virtual QPixmap pixmapFromUrl(int viewStyle) const;

    const std::vector<cv::Point2f> &validLaserPoints() const { return m_validLaserPoints; }

signals:

public slots:
    virtual bool findPattern(ZCalibrationPatternFinder *patternFinder);

protected:
    std::vector<cv::Point2f> m_laserPoints;

    std::vector<cv::Point2f> m_convexHullPoints;
    std::vector<cv::Point2f> m_validLaserPoints;
};

} // namespace Z3D

Q_DECLARE_METATYPE(Z3D::ZLTSCalibrationImage::Ptr) /// to use in QVariant
