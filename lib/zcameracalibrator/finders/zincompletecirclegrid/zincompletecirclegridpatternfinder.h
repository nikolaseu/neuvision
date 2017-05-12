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

class ZIncompleteCircleGridPatternFinder : public ZCalibrationPatternFinder
{
    Q_OBJECT

public:
    explicit ZIncompleteCircleGridPatternFinder(QObject *parent = 0);

    virtual QString name();

    virtual QWidget *configWidget();

    int maxColumns() const;
    int maxRows() const;

    bool isAsymmetricGrid() const;

    bool refinePatternPoints() const;

signals:
    void maxColumnsChanged(int columns);
    void maxRowsChanged(int rows);

    void isAsymmetricGridChanged(bool isAsymetric);

    void refinePatternPointsChanged(bool refinePatternPoints);

public slots:
    virtual bool findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &patternPoints);

    void setMaxColumns(int columns);
    void setMaxRows(int rows);

    void setIsAsymmetricGrid(bool isAsymmetric);

    void setRefinePatternPoints(bool refinePatternPoints);

protected slots:
    virtual void updateConfigHash();

protected:
    QWidget *m_configWidget;

    cv::Size m_completeBoardSize;

    bool m_isAsymmetricGrid;

    bool m_refinePatternPoints;
};

} // namespace Z3D
