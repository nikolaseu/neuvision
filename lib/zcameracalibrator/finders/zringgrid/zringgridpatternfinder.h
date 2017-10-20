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

class ZRingGridPatternFinder : public ZCalibrationPatternFinder
{
    Q_OBJECT

public:
    explicit ZRingGridPatternFinder(QObject *parent = nullptr);

    QString id() const override;
    QString name() const override;

    int maxColumns() const;
    int maxRows() const;

    bool refinePatternPoints() const;

signals:
    void maxColumnsChanged(int columns);
    void maxRowsChanged(int rows);

    void refinePatternPointsChanged(bool refinePatternPoints);

public slots:
    void setMaxColumns(int columns);
    void setMaxRows(int rows);

    void setRefinePatternPoints(bool refinePatternPoints);

protected slots:
    bool findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &patternPoints) override;
    void updateConfigHash() override;

protected:
    cv::Size m_completeBoardSize;

    bool m_refinePatternPoints;
};

} // namespace Z3D
