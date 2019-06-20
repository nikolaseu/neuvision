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

class ZRingGrid2PatternFinder : public ZCalibrationPatternFinder
{
    Q_OBJECT

    Q_PROPERTY(bool refinePatternPoints READ refinePatternPoints WRITE setRefinePatternPoints NOTIFY refinePatternPointsChanged)
    Q_PROPERTY(int blurSize READ blurSize WRITE setBlurSize NOTIFY blurSizeChanged)
    Q_PROPERTY(int cannyLowThreshold READ cannyLowThreshold WRITE setCannyLowThreshold NOTIFY cannyLowThresholdChanged)
    Q_PROPERTY(int cannyHighThreshold READ cannyHighThreshold WRITE setCannyHighThreshold NOTIFY cannyHighThresholdChanged)
    Q_PROPERTY(int cannyApertureSize READ cannyApertureSize WRITE setCannyApertureSize NOTIFY cannyApertureSizeChanged)

public:
    explicit ZRingGrid2PatternFinder(QObject *parent = nullptr);

    QString id() const override;
    QString name() const override;

    int maxColumns() const;
    int maxRows() const;

    bool refinePatternPoints() const;
    int blurSize() const;
    int cannyLowThreshold() const;
    int cannyHighThreshold() const;
    int cannyApertureSize() const;

signals:
    void maxColumnsChanged(int columns);
    void maxRowsChanged(int rows);

    void refinePatternPointsChanged(bool refinePatternPoints);
    void blurSizeChanged(int blurSize);
    void cannyLowThresholdChanged(int cannyLowThreshold);
    void cannyHighThresholdChanged(int cannyHighThreshold);
    void cannyApertureSizeChanged(int cannyApertureSize);

public slots:
    void setMaxColumns(int columns);
    void setMaxRows(int rows);

    void setRefinePatternPoints(bool refinePatternPoints);
    void setBlurSize(int blurSize);
    void setCannyLowThreshold(int cannyLowThreshold);
    void setCannyHighThreshold(int cannyHighThreshold);
    void setCannyApertureSize(int cannyApertureSize);

protected slots:
    bool findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &patternPoints) override;
    void updateConfigHash() override;

protected:
    cv::Size m_completeBoardSize = cv::Size(25, 25);
    bool m_refinePatternPoints = false;
    bool m_equalizeHistogram = false;
    int m_blurSize = 5;
    int m_cannyLowThreshold = 50;
    int m_cannyHighThreshold = 150;
    int m_cannyApertureSize = 3;
    float m_squareSize = 1.0f;
    float m_maxRectifiedDistance = m_squareSize / 2.f;
};

} // namespace Z3D
