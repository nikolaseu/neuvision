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

#include <Z3DCameraAcquisition>

#include <QObject>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace Z3D
{


class ScanImageViewerFilter : public QObject
{
    Q_OBJECT

public:
    explicit ScanImageViewerFilter(QObject *parent = 0);
    virtual ~ScanImageViewerFilter();

signals:
    void imageProcessed(ZImageGrayscale::Ptr image);

public slots:
    virtual ZImageGrayscale::Ptr processImage(ZImageGrayscale::Ptr image);
};


class LineSubtractionScanImageFilter : public ScanImageViewerFilter
{
    Q_OBJECT
public:
    explicit LineSubtractionScanImageFilter(QObject *parent = 0);
    virtual ~LineSubtractionScanImageFilter();

public slots:
    ZImageGrayscale::Ptr processImage(ZImageGrayscale::Ptr originalImage);
};


class LineSubtractionRansacScanImageViewerFilter : public ScanImageViewerFilter
{
    Q_OBJECT
public:
    explicit LineSubtractionRansacScanImageViewerFilter(QObject *parent = 0);
    virtual ~LineSubtractionRansacScanImageViewerFilter();

public slots:
    ZImageGrayscale::Ptr processImage(ZImageGrayscale::Ptr originalImage);

protected:
    pcl::SACSegmentation<pcl::PointXYZ> lineSeg;
    pcl::ModelCoefficients::Ptr lineCoefficients;
    pcl::PointIndices::Ptr lineInliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;
    bool m_lineOptimizeCoeffs;
    double m_lineThresholdDistance;
    std::vector<cv::Point2i> imgRowVector;
};


class UnsharpMaskingScanImageViewerFilter : public ScanImageViewerFilter
{
    Q_OBJECT
public:
    explicit UnsharpMaskingScanImageViewerFilter(QObject *parent = 0);
    virtual ~UnsharpMaskingScanImageViewerFilter();

public slots:
    ZImageGrayscale::Ptr processImage(ZImageGrayscale::Ptr originalImage);

};


} // namespace Z3D
