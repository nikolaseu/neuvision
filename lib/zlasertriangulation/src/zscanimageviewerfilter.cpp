#include "zscanimageviewerfilter.h"

namespace Z3D
{

ScanImageViewerFilter::ScanImageViewerFilter(QObject *parent)
    : QObject(parent)
{
}

ScanImageViewerFilter::~ScanImageViewerFilter()
{

}

ZImageGrayscale::Ptr ScanImageViewerFilter::processImage(ZImageGrayscale::Ptr image)
{
    /// dummy processing.
    /// copy image to avoid modifying the original when the colormap is applied
    ZImageGrayscale::Ptr image2 = image->clone();
    emit imageProcessed(image2);
    return image2;
}

LineSubtractionScanImageFilter::LineSubtractionScanImageFilter(QObject *parent)
    : ScanImageViewerFilter(parent)
{

}

LineSubtractionScanImageFilter::~LineSubtractionScanImageFilter()
{

}

ZImageGrayscale::Ptr LineSubtractionScanImageFilter::processImage(ZImageGrayscale::Ptr originalImage)
{
    ZImageGrayscale::Ptr image = originalImage->clone();

    cv::Mat m_img = image->cvMat();

    if (m_img.type() == CV_16UC1) {
        cv::medianBlur(m_img, m_img, 3);
    } else {
        qWarning() << "only CV_16UC1 images implemented for the moment...";
        return ZImageGrayscale::Ptr(0);
    }

    std::vector<cv::Point2i> rowVector;
    rowVector.reserve(m_img.cols);
    for (int r=0; r<m_img.rows; ++r) {
        rowVector.clear();

        if (m_img.type() == CV_16UC1) {
            const unsigned short* data = m_img.ptr<unsigned short>(r);
            const int step = 2;
            for (int c = 0; c < m_img.cols; c += step) {
                const unsigned short* value = data;
                if (*value)
                    rowVector.push_back(cv::Point2i(c, *value));
                data += step;
            }
        } else {
            qWarning() << "only CV_16UC1 images implemented for the moment...";
            return ZImageGrayscale::Ptr(0);
        }

        /** OpenCV fitLine
          void fitLine(InputArray points, OutputArray line, int distType, double param, double reps, double aeps)
            * points – Input vector of 2D or 3D points, stored in
              std::vector<> or Mat.
            * line – Output line parameters. In case of 2D fitting, it should
              be a vector of 4 elements (like Vec4f) - (vx, vy, x0, y0),
              where (vx, vy) is a normalized vector collinear to the line and
              (x0, y0) is a point on the line. In case of 3D fitting, it should
              be a vector of 6 elements (like Vec6f) - (vx, vy, vz, x0, y0, z0),
              where (vx, vy, vz) is a normalized vector collinear to the line
              and (x0, y0, z0) is a point on the line.
            * distType – Distance used by the M-estimator (see the discussion below).
            * param – Numerical parameter ( C ) for some types of distances. If it is 0, an optimal value is chosen.
            * reps – Sufficient accuracy for the radius (distance between the coordinate origin and the line).
            * aeps – Sufficient accuracy for the angle. 0.01 would be a good default value for reps and aeps.
        */
        if (rowVector.size() > 3) {
            cv::Vec4f lineCoeffs;
            cv::fitLine(rowVector, lineCoeffs,
#ifdef CV_VERSION_EPOCH
                        CV_DIST_L2
#else
                        cv::DIST_L2
#endif
                        , 0, 0.01, 0.01);

            float vx = lineCoeffs[0];
            float vy = lineCoeffs[1];
            float x0 = lineCoeffs[2];
            float y0 = lineCoeffs[3];

            //if (r==0)
            //    qDebug() << vx << vy << x0 << y0;

            /// subtract line
            if (m_img.type() == CV_16UC1) {
                unsigned short* data = m_img.ptr<unsigned short>(r);
                for (int c=0; c<m_img.cols; ++c) {
                    if (*data) {
                        float dx = x0 - c;
                        float s = dx/vx;
                        *data = (unsigned short)32000 + *data - (unsigned short) (y0 - s * vy);
                    }
                    data++;
                }
            } else {
                qWarning() << "only CV_16UC1 images implemented for the moment...";
                return ZImageGrayscale::Ptr(0);
            }
        }
    }

    emit imageProcessed(image);

    return image;
}

LineSubtractionRansacScanImageViewerFilter::LineSubtractionRansacScanImageViewerFilter(QObject *parent)
    : ScanImageViewerFilter(parent)
{
    lineCoefficients = pcl::ModelCoefficients::Ptr( new pcl::ModelCoefficients );
    lineInliers = pcl::PointIndices::Ptr( new pcl::PointIndices );
    inputCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ> );

     /// Optional
    m_lineOptimizeCoeffs = true;
    lineSeg.setOptimizeCoefficients(m_lineOptimizeCoeffs);
    /// Mandatory
    //lineSeg.setModelType(pcl::SACMODEL_CIRCLE2D);
    lineSeg.setModelType(pcl::SACMODEL_LINE);

    //pcl::SAC_RANSAC
    //pcl::SAC_LMEDS
    //lineSeg.setMethodType(pcl::SAC_LMEDS);
    lineSeg.setMethodType(pcl::SAC_RANSAC);

    m_lineThresholdDistance = 1.0 * 64; // 2 px y 64 subpixel resolution
    lineSeg.setDistanceThreshold(m_lineThresholdDistance);
}

LineSubtractionRansacScanImageViewerFilter::~LineSubtractionRansacScanImageViewerFilter()
{

}

ZImageGrayscale::Ptr LineSubtractionRansacScanImageViewerFilter::processImage(ZImageGrayscale::Ptr originalImage)
{
    ZImageGrayscale::Ptr image = originalImage->clone();

    cv::Mat m_img = image->cvMat();

    if (m_img.type() == CV_16UC1) {
        //cv::medianBlur(m_img, m_img, 3);
    } else {
        qWarning() << "only CV_16UC1 images implemented for the moment...";
        return ZImageGrayscale::Ptr(0);
    }

    imgRowVector.reserve(m_img.cols);
    for (int r=0; r<m_img.rows; ++r) {
        imgRowVector.clear();

        if (m_img.type() == CV_16UC1) {
            const unsigned short* value = m_img.ptr<unsigned short>(r);
            const int step = 4;
            for (int c = 0; c < m_img.cols; c += step) {
                //const unsigned short* value = value;
                if (*value)
                    imgRowVector.push_back(cv::Point2i(c, *value));
                value += step;
            }
        } else {
            qWarning() << "only CV_16UC1 images implemented for the moment...";
            return ZImageGrayscale::Ptr(0);
        }

        if (imgRowVector.size() > 3) {
            inputCloud->resize(imgRowVector.size());
            for (int size = imgRowVector.size(), n = 0; n<size; ++n) {
                const cv::Point2i &point = imgRowVector[n];
                pcl::PointXYZ &pointPcl = inputCloud->points[n];
                pointPcl.x = point.x;
                pointPcl.y = point.y;
                pointPcl.z = 0;
            }

            lineSeg.setInputCloud(inputCloud);
            lineSeg.segment(*lineInliers, *lineCoefficients);

            /*
            /// debug circle coeffs
            if (r==0)
                qDebug() << lineCoefficients->values[0]
                        << lineCoefficients->values[1]
                        << lineCoefficients->values[2]
                        << lineCoefficients->values[3]
                        << lineCoefficients->values[4]
                        << lineCoefficients->values[5];
            */

            /// subtract line
            float x0 = lineCoefficients->values[0];
            float y0 = lineCoefficients->values[1];
            float vx = lineCoefficients->values[3];
            float vy = lineCoefficients->values[4];

            //if (r==0)
            //    qDebug() << vx << vy << x0 << y0;

            /// subtract line
            if (m_img.type() == CV_16UC1) {
                unsigned short* data = m_img.ptr<unsigned short>(r);
                for (int c=0; c<m_img.cols; ++c) {
                    if (*data) {
                        float dx = x0 - c;
                        float s = dx/vx;
                        *data = (unsigned short)32000 + *data - (unsigned short) (y0 - s * vy);
                    }
                    data++;
                }
            } else {
                qWarning() << "only CV_16UC1 images implemented for the moment...";
                return ZImageGrayscale::Ptr(0);
            }

            /// subtract line
            /*
                if (m_img.type() == CV_16UC1) {
                    unsigned short* data = m_img.ptr<unsigned short>(r);
                    for (int c=0; c<m_img.cols; ++c) {
                        if (*data) {
                            float dx = x0 - c;
                            float s = dx/vx;
                            *data = (unsigned short)32000 + *data - (unsigned short) (y0 - s * vy);
                        }
                        data++;
                    }
                } else {
                    qWarning() << "only CV_16UC1 images implemented for the moment...";
                    return;
                }*/
        }
    }

    emit imageProcessed(image);

    return image;
}

UnsharpMaskingScanImageViewerFilter::UnsharpMaskingScanImageViewerFilter(QObject *parent)
    : ScanImageViewerFilter(parent)
{

}

UnsharpMaskingScanImageViewerFilter::~UnsharpMaskingScanImageViewerFilter()
{

}

ZImageGrayscale::Ptr UnsharpMaskingScanImageViewerFilter::processImage(ZImageGrayscale::Ptr originalImage)
{
    ZImageGrayscale::Ptr image = originalImage->clone();

    cv::Mat m_img = image->cvMat();

    if (m_img.type() == CV_16UC1) {
        cv::medianBlur(m_img, m_img, 3);
    } else {
        qWarning() << "only CV_16UC1 images implemented for the moment...";
        return ZImageGrayscale::Ptr(0);
    }

    if (m_img.channels() == 1) {

        /*if (m_img.type() != CV_8U) {

                m_img.convertTo(m_img, CV_8UC1);
            }*/

        cv::Mat medianImg;
        /// calculate median
        /// medianBlur only works for small kernels
        /*const int medianWinSize = 3;
            cv::medianBlur(m_img, medianImg, medianWinSize);*/

        /** OpenCV blur
             Various border types, image boundaries are denoted with '|'
                * BORDER_REPLICATE:     aaaaaa|abcdefgh|hhhhhhh
                * BORDER_REFLECT:       fedcba|abcdefgh|hgfedcb
                * BORDER_REFLECT_101:   gfedcb|abcdefgh|gfedcba
                * BORDER_WRAP:          cdefgh|abcdefgh|abcdefg
                * BORDER_CONSTANT:      iiiiii|abcdefgh|iiiiiii  with some specified 'i'
             */
        const int kernelSize = 16;
        cv::blur(m_img, medianImg, cv::Size(kernelSize,kernelSize), cv::Point(-1,-1), cv::BORDER_REPLICATE);


        /// fill empty values with median
        cv::Mat mask = m_img == 0;
        medianImg.copyTo(m_img, mask);
/*
        float alpha = 0.6f;
        float beta = 1.f - alpha;

        /// substract median image
        m_img = alpha * m_img - beta * medianImg;
*/
        m_img = m_img + (32768 - medianImg); /// center in unsigned 16bits = 32768
    } else {
        qCritical() << "unsupported image";
        return ZImageGrayscale::Ptr(0);
    }

    emit imageProcessed(image);

    return image;
}

} // namespace Z3D
