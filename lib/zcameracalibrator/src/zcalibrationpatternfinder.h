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

#include "zcameracalibrator_global.h"

#include <QObject>
#include <QPointer>
#include <QSharedPointer>

#include <opencv2/core/core.hpp>

namespace Z3D
{

class Z3D_CAMERACALIBRATOR_SHARED_EXPORT ZCalibrationPatternFinder : public QObject
{
    Q_OBJECT

    Q_PROPERTY(int   columns   READ columns   WRITE setColumns   NOTIFY columnsChanged)
    Q_PROPERTY(int   rows      READ rows      WRITE setRows      NOTIFY rowsChanged)
    Q_PROPERTY(float colWidth  READ colWidth  WRITE setColWidth  NOTIFY colWidthChanged)
    Q_PROPERTY(float rowHeight READ rowHeight WRITE setRowHeight NOTIFY rowHeightChanged)

public:
    typedef std::shared_ptr<ZCalibrationPatternFinder> Ptr;
    typedef QPointer<ZCalibrationPatternFinder> WeakPtr;

    explicit ZCalibrationPatternFinder(QObject *parent = nullptr);

    virtual QString id() const = 0;
    virtual QString name() const = 0;

    int columns() const;
    int rows() const;
    double colWidth() const;
    double rowHeight() const;

    QString configHash() const;

signals:
    void columnsChanged(int columns);
    void rowsChanged(int rows);
    void colWidthChanged(double colWidth);
    void rowHeightChanged(double rowHeight);

    void configHashChanged(QString newConfigHash);

public slots:
    void setColumns(int columns);
    void setRows(int rows);
    void setColWidth(double colWidth);
    void setRowHeight(double rowHeight);

    bool findCalibrationPattern(
            cv::Mat image,
            std::vector<cv::Point2f> &corners,
            std::vector<cv::Point3f> &objectPoints,
            std::vector<cv::Point3f> &patternPoints);

protected slots:
    virtual bool findCalibrationPattern(
            cv::Mat image,
            std::vector<cv::Point2f> &corners,
            std::vector<cv::Point3f> &patternPoints) = 0;

    /// returns the hash of the parameters of this class, includign pattern type, id, size, etc
    QString getBaseHash();

    /// subclasses must implement this function to generate a unique "hash" for the current configuration
    virtual void updateConfigHash() = 0;

protected:
    QString m_configHash;

    cv::Size m_boardSize;

private:
    double m_colWidth;
    double m_rowHeight;
};

} // namespace Z3D
