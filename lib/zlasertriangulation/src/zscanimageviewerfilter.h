#ifndef SCANIMAGEVIEWERFILTER_H
#define SCANIMAGEVIEWERFILTER_H

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
    virtual void imageProcessed(ZImageGrayscale::Ptr image);

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

#endif // SCANIMAGEVIEWERFILTER_H
