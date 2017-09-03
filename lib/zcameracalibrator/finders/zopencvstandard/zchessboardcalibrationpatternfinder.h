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

#include "zcalibrationpatternfinder.h"

namespace Z3D
{

class ZChessboardCalibrationPatternFinder : public ZCalibrationPatternFinder
{
    Q_OBJECT

    /// find chessboard corners options
    Q_PROPERTY(bool normalizeImage
               READ normalizeImage
               WRITE setNormalizeImage
               NOTIFY normalizeImageChanged)
    Q_PROPERTY(bool useAdaptiveThreshold
               READ useAdaptiveThreshold
               WRITE setUseAdaptiveThreshold
               NOTIFY useAdaptiveThresholdChanged)
    Q_PROPERTY(bool useFastCheck
               READ useFastCheck
               WRITE setUseFastCheck
               NOTIFY useFastCheckChanged)
    Q_PROPERTY(bool filterQuads
               READ filterQuads
               WRITE setFilterQuads
               NOTIFY filterQuadsChanged)

    /// corner sub pix options
    Q_PROPERTY(int subPixWinWidth
               READ subPixWinWidth
               WRITE setSubPixWinWidth
               NOTIFY subPixWinWidthChanged)
    Q_PROPERTY(int subPixWinHeight
               READ subPixWinHeight
               WRITE setSubPixWinHeight
               NOTIFY subPixWinHeightChanged)
    Q_PROPERTY(int subPixZeroZoneWinWidth
               READ subPixZeroZoneWinWidth
               WRITE setSubPixZeroZoneWinWidth
               NOTIFY subPixZeroZoneWinWidthChanged)
    Q_PROPERTY(int subPixZeroZoneWinHeight
               READ subPixZeroZoneWinHeight
               WRITE setSubPixZeroZoneWinHeight
               NOTIFY subPixZeroZoneWinHeightChanged)
    Q_PROPERTY(int subPixMaxIter
               READ subPixMaxIter
               WRITE setSubPixMaxIter
               NOTIFY subPixMaxIterChanged)
    Q_PROPERTY(double subPixEpsilon
               READ subPixEpsilon
               WRITE setSubPixEpsilon
               NOTIFY subPixEpsilonChanged)

public:
    explicit ZChessboardCalibrationPatternFinder(QObject *parent = 0);

    QString name() const override;

    bool normalizeImage() const;
    bool useAdaptiveThreshold() const;
    bool useFastCheck() const;
    bool filterQuads() const;

    int subPixWinWidth() const;
    int subPixWinHeight() const;
    int subPixZeroZoneWinWidth() const;
    int subPixZeroZoneWinHeight() const;
    int subPixMaxIter() const;
    double subPixEpsilon() const;

signals:
    void normalizeImageChanged(bool arg);
    void useAdaptiveThresholdChanged(bool arg);
    void useFastCheckChanged(bool arg);
    void filterQuadsChanged(bool arg);

    void subPixWinWidthChanged(int arg);
    void subPixWinHeightChanged(int arg);
    void subPixZeroZoneWinWidthChanged(int arg);
    void subPixZeroZoneWinHeightChanged(int arg);
    void subPixMaxIterChanged(int arg);
    void subPixEpsilonChanged(double arg);

public slots:
    void setNormalizeImage(bool normalizeImage);
    void setUseAdaptiveThreshold(bool useAdaptiveThreshold);
    void setUseFastCheck(bool useFastCheck);
    void setFilterQuads(bool filterQuads);

    void setSubPixWinWidth(int arg);
    void setSubPixWinHeight(int arg);
    void setSubPixZeroZoneWinWidth(int arg);
    void setSubPixZeroZoneWinHeight(int arg);
    void setSubPixMaxIter(int arg);
    void setSubPixEpsilon(double arg);

    bool findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &patternPoints) override;

protected slots:
    void updateConfigHash() override;

protected:
    /// findChessboardCorners flags
    int m_findChessboardFlags;

    /// cornerSubPix config
    int m_subPixWinWidth;
    int m_subPixWinHeight;
    int m_subPixZeroZoneWinWidth;
    int m_subPixZeroZoneWinHeight;
    int m_subPixMaxIter;
    double m_subPixEpsilon;
};

} // namespace Z3D
