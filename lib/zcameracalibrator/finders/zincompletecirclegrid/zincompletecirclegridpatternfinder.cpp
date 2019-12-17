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

#include "zincompletecirclegridpatternfinder.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <QDebug>

//#define DEBUG_RECTANGLE_DETECTOR
//#define DEBUG_PERSPECTIVE_TRANSFORMATION

#if defined(DEBUG_RECTANGLE_DETECTOR) || defined(DEBUG_PERSPECTIVE_TRANSFORMATION)
#  include <QDir>
#  include "opencv2/highgui.hpp" //to use imwrite, imshow
#endif


namespace Z3D
{

static bool  m_equalizeHistogram = (false);
static int   m_cannyLowThreshold = (20);
static int   m_cannyHighThreshold = (50); // should be close to three times the lower threshold
static int   m_cannyApertureSize = (3);
static float m_minAreaProportion = (0.005f); /// min area is width/10 and height/10 = 1% of total
static float m_squareSize = (1.0f);
static float m_maxRectifiedDistance = (m_squareSize / 2.0f);
// rectangle validator
static float m_minCos = (-0.3f);
static float m_maxCos = (0.45f);

/**
 * finds a cosine of angle between vectors from pt0->pt1 and pt0->pt2
 */
float angle(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt0)
{
    float dx1 = pt1.x - pt0.x;
    float dy1 = pt1.y - pt0.y;
    float dx2 = pt2.x - pt0.x;
    float dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10f);
}


ZIncompleteCircleGridPatternFinder::ZIncompleteCircleGridPatternFinder(QObject *parent)
    : ZCalibrationPatternFinder(parent)
    , m_completeBoardSize(31, 31)
    , m_isAsymmetricGrid(false)
    , m_refinePatternPoints(false)
{
    QObject::connect(this, &ZIncompleteCircleGridPatternFinder::maxColumnsChanged,
                     this, &ZIncompleteCircleGridPatternFinder::updateConfigHash);
    QObject::connect(this, &ZIncompleteCircleGridPatternFinder::maxRowsChanged,
                     this, &ZIncompleteCircleGridPatternFinder::updateConfigHash);
    QObject::connect(this, &ZIncompleteCircleGridPatternFinder::isAsymmetricGridChanged,
                     this, &ZIncompleteCircleGridPatternFinder::updateConfigHash);
    QObject::connect(this, &ZIncompleteCircleGridPatternFinder::refinePatternPointsChanged,
                     this, &ZIncompleteCircleGridPatternFinder::updateConfigHash);

    /// generate current config hash
    updateConfigHash();

#if defined(DEBUG_RECTANGLE_DETECTOR) || defined(DEBUG_PERSPECTIVE_TRANSFORMATION)
    /// create folder where the debug images will be saved
    QDir::current().mkpath("tmp/debug");
#endif
}

QString ZIncompleteCircleGridPatternFinder::id() const
{
    return QString(metaObject()->className());
}

QString ZIncompleteCircleGridPatternFinder::name() const
{
    return QLatin1String("Incomplete circle grid");
}

int ZIncompleteCircleGridPatternFinder::maxColumns() const
{
    return m_completeBoardSize.width;
}

int ZIncompleteCircleGridPatternFinder::maxRows() const
{
    return m_completeBoardSize.height;
}

bool ZIncompleteCircleGridPatternFinder::isAsymmetricGrid() const
{
    return m_isAsymmetricGrid;
}

bool ZIncompleteCircleGridPatternFinder::refinePatternPoints() const
{
    return m_refinePatternPoints;
}

bool ZIncompleteCircleGridPatternFinder::findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &patternPoints)
{
    cv::Mat gray;

    /// convert to grayscale when neccessary
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY); //CV_BGR2GRAY);
    } else if (image.channels() == 1) {
        if (m_equalizeHistogram) {
            /// don't modify original image
            gray = image.clone();
        } else {
            gray = image;
        }
    } else {
        qWarning() << "unknown image type";
        return false;
    }

    if (m_equalizeHistogram) {
        /// normalize image
        cv::equalizeHist(gray, gray);
    }

    /// find edges
    /// blur image a little to avoid getting noisy edges
    cv::Mat blurredGray;
    cv::blur(gray, blurredGray, cv::Size(5,5));
    cv::Mat bw;
    cv::Canny(blurredGray, bw, m_cannyLowThreshold, m_cannyHighThreshold, m_cannyApertureSize);

    /// close shapes
    cv::Mat dilateErodeKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
    cv::dilate(bw, bw, dilateErodeKernel);
    cv::erode(bw, bw, dilateErodeKernel);

    /// find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(bw, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const int minArea = int(gray.size().area() * m_minAreaProportion);

#ifdef DEBUG_RECTANGLE_DETECTOR
    static QAtomicInt atomicInt = 0;
    const int currentIndex = atomicInt.fetchAndAddAcquire(1);
    //cv::imwrite(qPrintable(QString("%1gray.jpg").arg(tmpint)), gray);
    cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_bw.bmp").arg(currentIndex)), bw!=0);
    cv::Mat contoursImage;

    cv::cvtColor(image, contoursImage, cv::COLOR_GRAY2RGB);
    for (size_t i = 0; i < contours.size(); i++) {
        cv::drawContours(contoursImage, contours, int(i), cv::Scalar(0,0,255), 2);

//        /// debug individual contours
//        cv::Mat thisContourImage;
//        cv::cvtColor(image, thisContourImage, cv::COLOR_GRAY2RGB);
//        cv::drawContours(thisContourImage, contours, int(i), cv::Scalar(0,0,255), 2);
//        cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_contour_%2.bmp").arg(currentIndex).arg(i)), thisContourImage);
    }
    cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_contours.bmp").arg(currentIndex)), contoursImage);

    cv::cvtColor(image, contoursImage, cv::COLOR_GRAY2RGB);
    for (size_t i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i], false) >= minArea) {
            cv::drawContours(contoursImage, contours, int(i), cv::Scalar(0,0,255), 5);
        }
    }
    cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_contours_valid.bmp").arg(currentIndex)), contoursImage);
#endif

    int iRect = 0;
    std::vector<cv::Point> approx;
    for (size_t i = 0; i < contours.size(); i++) {
        /// skip small objects
        if (cv::contourArea(contours[i], false) < minArea) {
            continue;
        }

        /// approximate contour with accuracy proportional to the contour perimeter
        cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true)*0.02, true);

        /// number of vertices of polygonal curve
        const size_t vtc = approx.size();

        /// skip non-rectangular shapes
        if (vtc != 4) {
            continue;
        }

        /// skip non-convex objects
        if (!cv::isContourConvex(approx)) {
            continue;
        }

        /// get the cosines of all corners
        std::vector<float> cos;
        for (size_t j = 2; j < vtc+1; j++) {
            cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));
        }

        /// sort ascending the cosine values
        std::sort(cos.begin(), cos.end());

        /// get the lowest and the highest cosine
        const float mincos = cos.front();
        const float maxcos = cos.back();

        /// use the degrees obtained above to determine valid rectangles
        if (mincos >= m_minCos && maxcos <= m_maxCos) {
            /// we found a rectangle
            /// get the minimum rect containing the detected rectangle
            const cv::Point &point = approx[0];
            int minX = point.x
              , minY = point.y
              , maxX = point.x
              , maxY = point.y;

            for (size_t c=1; c<vtc; ++c) {
                const cv::Point &point = approx[c];
                if (point.x < minX)
                    minX = point.x;
                if (point.y < minY)
                    minY = point.y;
                if (point.x > maxX)
                    maxX = point.x;
                if (point.y > maxY)
                    maxY = point.y;
            }

            /// try to find standard circle grid inside rectangle
            /// use only the section of the image that matters
            cv::Mat roi = gray(cv::Rect(minX, minY, maxX-minX, maxY-minY));

            /// remove outside of the rectangle to avoid false detections
            cv::Mat roiMask2 = cv::Mat::zeros(roi.rows, roi.cols, roi.type());
            cv::Point startPoint(minX, minY);
            for (size_t c=0; c<vtc; ++c) {
                approx[c] -= startPoint;
            }

            /// reduce the polygon to avoid including the rectangle edges.
            /// but it's not really necessary ...
//            const float diagSize = std::sqrtf(std::powf(m_boardSize.width, 2) + std::powf(m_boardSize.height, 2));
//            const float relativeMotion = 0.2f / diagSize;
//            for (size_t c=0; c<vtc; ++c) {
//                const size_t oppositeIndex = (c + 2) % vtc;
//                approx[c] += relativeMotion * (approx[oppositeIndex] - approx[c]);
//            }

            /// create mask
            cv::fillConvexPoly(roiMask2, approx, cv::Scalar(1));

            double minVal, maxVal;
            cv::minMaxLoc(roi, &minVal, &maxVal);
            cv::Mat roiClean = cv::Mat(roi.rows, roi.cols, roi.type(), cv::Scalar(maxVal));
            roi.copyTo(roiClean, roiMask2);

            /// since we assume inside the rectangle there's a circle grid only,
            /// we could apply a binary threshold, but it's not really necessary ...
//            cv::threshold(roiClean, roiClean, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

            const double rectArea = cv::contourArea(approx, false);

            cv::SimpleBlobDetector::Params blobDetectorParams;
            blobDetectorParams.filterByArea = true;
            blobDetectorParams.maxArea = 0.7f * float(rectArea / m_boardSize.area()); // cannot be bigger than this otherwise they would overlap
            blobDetectorParams.minArea = 0.05f * float(rectArea / m_boardSize.area()); // cannot be too small either

            cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(blobDetectorParams);

#ifdef DEBUG_RECTANGLE_DETECTOR
            qDebug() << "board size:" << m_boardSize.width << "x" << m_boardSize.height
                     << "rect area:" << rectArea
                     << "min blob area:" << blobDetectorParams.minArea
                     << "max blob area:" << blobDetectorParams.maxArea;

            //cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_roi.bmp").arg(currentIndex)), roiClean);

            /// test blobDetector
            cv::Mat roiMaskKeypoints;
            cv::cvtColor(roiClean, roiMaskKeypoints, cv::COLOR_GRAY2BGR);
            std::vector<cv::KeyPoint> keypoints;
            blobDetector->detect(roiClean, keypoints);
            for (size_t i = 0; i < keypoints.size(); i++) {
                cv::circle(roiMaskKeypoints, keypoints[i].pt, 3, cv::Scalar(0, 255, 0), 3);
            }
            cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_roiKeypoints_%2.bmp").arg(currentIndex).arg(i)), roiMaskKeypoints);
#endif

#if CV_MAJOR_VERSION >= 4
            /// Relax parameters for circle grid finder
            cv::CirclesGridFinderParameters circleGridParameters;
            circleGridParameters.minDensity = 0; //10 is default but is not working for my calibration plate
#endif

            if (cv::findCirclesGrid(roiClean, m_boardSize, corners,
                                    //cv::CALIB_CB_CLUSTERING |
                                    m_isAsymmetricGrid ? cv::CALIB_CB_ASYMMETRIC_GRID : cv::CALIB_CB_SYMMETRIC_GRID,
                                    blobDetector
#if CV_MAJOR_VERSION >= 4
                                    , circleGridParameters
#endif
                                    ))
            {
                /// the standard circle grid was found
                /// return corners to original image coordinates
                cv::Point2f origin(minX, minY);
                for (std::vector<cv::Point2f>::iterator c = corners.begin(); c != corners.end(); ++c) {
                    *c += origin;
                }

                /// get coordinates of ideal points, the indices inside the standard grid
                std::vector<cv::Point2f> idealPoints;
                /// generate world object coordinates
                for (int i=0; i<m_boardSize.height; ++i) {
                    for (int j=0; j<m_boardSize.width; ++j) {
                        if (m_isAsymmetricGrid)
                            idealPoints.push_back(cv::Point2f((2*j + i % 2)*m_squareSize, i*m_squareSize));
                        else
                            idealPoints.push_back(cv::Point2f(j*m_squareSize, i*m_squareSize));
                    }
                }

                /// detect all possible pattern points
                std::vector<cv::KeyPoint> keypoints;

                blobDetector->detect(gray, keypoints);
                std::vector<cv::Point2f> detectedPatternPoints;
                for (size_t i = 0; i < keypoints.size(); i++) {
                    detectedPatternPoints.push_back (keypoints[i].pt);
                }

                /// using the standard circle grid found, rectify all the possible points,
                /// get how they would be seen if picture was taken from the front

                /// compute homography
                cv::Mat homography = cv::findHomography(cv::Mat(corners), cv::Mat(idealPoints), 0);
                cv::Mat rectifiedDetectedPatternPointsMat;
                cv::transform(detectedPatternPoints, rectifiedDetectedPatternPointsMat, homography);
                std::vector<cv::Point2f> rectifiedDetectedPatternPoints;
                cv::convertPointsFromHomogeneous(rectifiedDetectedPatternPointsMat, rectifiedDetectedPatternPoints);

                /// now that we have the rectified points, search for possible valid points
                /// using the complete/max size of the grid.
                /// use the ones that are near the ideal position
                cv::flann::LinearIndexParams flannIndexParams;
                cv::flann::Index flannIndex(cv::Mat(rectifiedDetectedPatternPoints).reshape(1), flannIndexParams);

                std::vector<cv::Point2f> centers;
                std::vector<cv::Point3f> objectCenters;

                int extraRows = (m_completeBoardSize.height - m_boardSize.height) / 2;
                int extraCols = (m_completeBoardSize.width - m_boardSize.width) / 2;

                int i, j;
                for( int i2 = 0; i2 < m_completeBoardSize.height; i2++ ) {
                    /// first index always coordinate origin
                    if (i2 >= m_boardSize.height+extraRows)
                        i = m_boardSize.height+extraRows - i2 - 1;
                    else
                        i = i2;

                    for( int j2 = 0; j2 < m_completeBoardSize.width; j2++ ) {

                        /// first index always coordinate origin
                        if (j2 >= m_boardSize.width+extraCols)
                            j = m_boardSize.width+extraCols - j2 - 1;
                        else
                            j = j2;

//                        qDebug() << "looking for point (" << i << "," << j << ")";

                        cv::Point2f idealPt;
                        if (m_isAsymmetricGrid)
                            idealPt = cv::Point2f((2*j + i % 2)*m_squareSize, i*m_squareSize);
                        else
                            idealPt = cv::Point2f(j*m_squareSize, i*m_squareSize);

                        std::vector<float> query = cv::Mat(idealPt);
                        size_t knn = 1;
                        std::vector<int> indices(knn);
                        std::vector<float> dists(knn);
                        flannIndex.knnSearch(query, indices, dists, int(knn), cv::flann::SearchParams());

                        if (dists[0] < m_maxRectifiedDistance) {
                            /// found valid point
                            if (m_refinePatternPoints) {
                                /// try to refine the search using a perspective rectified image
                                /// the image should display the area around the detected point
                                /// from a frontal  point of view
                                const cv::Point2f &center = rectifiedDetectedPatternPoints.at(indices[0]);
                                std::vector<cv::Point2f> rectCornerPoints(4);
                                /// how much of the area around the point should we use.
                                /// 1 == the other corners are visible
                                /// 0 == center point of the circle, nothing visible
                                static float m_refinementZoomArea = 0.4f;
                                /// top left
                                rectCornerPoints[0] = center + m_refinementZoomArea * cv::Point2f(-m_squareSize, -m_squareSize);
                                /// top right
                                rectCornerPoints[1] = center + m_refinementZoomArea * cv::Point2f(m_squareSize, -m_squareSize);
                                /// bottom right
                                rectCornerPoints[2] = center + m_refinementZoomArea * cv::Point2f(m_squareSize, m_squareSize);
                                /// bottom left
                                rectCornerPoints[3] = center + m_refinementZoomArea * cv::Point2f(-m_squareSize, m_squareSize);

                                cv::Mat idealToImgHomography = cv::findHomography(cv::Mat(idealPoints), cv::Mat(corners), 0);
                                cv::Mat imageRectCornersPointsMat;
                                cv::transform(rectCornerPoints, imageRectCornersPointsMat, idealToImgHomography);
                                std::vector<cv::Point2f> imageRectCornersPoints;
                                cv::convertPointsFromHomogeneous(imageRectCornersPointsMat, imageRectCornersPoints);

                                /// destination image
                                static const int m_refinementImgSize = 100;
                                cv::Mat imgRectifiedPatternPoint = cv::Mat::zeros(int(m_refinementImgSize*m_squareSize), int(m_refinementImgSize*m_squareSize), gray.type());

                                /// corners of the destination image
                                std::vector<cv::Point2f> quad_pts;
                                quad_pts.push_back(cv::Point2f(0, 0));
                                quad_pts.push_back(cv::Point2f(imgRectifiedPatternPoint.cols, 0));
                                quad_pts.push_back(cv::Point2f(imgRectifiedPatternPoint.cols, imgRectifiedPatternPoint.rows));
                                quad_pts.push_back(cv::Point2f(0, imgRectifiedPatternPoint.rows));

                                /// get transformation matrix
                                cv::Mat invPerspectiveTransf = cv::getPerspectiveTransform(quad_pts, imageRectCornersPoints);
                                //cv::Mat perspectiveTransf = cv::getPerspectiveTransform(imageRectCornersPoints, quad_pts);

                                /// apply perspective transformation
                                cv::warpPerspective(gray,
                                                    imgRectifiedPatternPoint,
                                                    invPerspectiveTransf,
                                                    imgRectifiedPatternPoint.size(),
                                                    cv::WARP_INVERSE_MAP | cv::INTER_LINEAR); // cv::INTER_CUBIC

                                /// smooth image
                                static const int blurSize = 9;
                                static const int blurSigma = 3;
                                cv::GaussianBlur(imgRectifiedPatternPoint, imgRectifiedPatternPoint, cv::Size(blurSize, blurSize), blurSigma, blurSigma);

#if 0
                                std::vector<cv::Vec3f> circles;
                                cv::HoughCircles(imgRectifiedPatternPoint, circles, cv::HOUGH_GRADIENT,
                                                 2,
                                                 m_refinementImgSize/2,
                                                 200,
                                                 100,
                                                 m_refinementImgSize/10); // min radius

                                cv::Mat img = cv::Mat::zeros( imgRectifiedPatternPoint.size(), CV_8UC3 );
                                cv::cvtColor(imgRectifiedPatternPoint, img, cv::COLOR_GRAY2RGB);
                                for( size_t i = 0; i < 1/*circles.size()*/; i++ ) {


                                    cv::Point2f detectedPoint(circles[i][0], circles[i][1]);
                                    std::vector<cv::Point2f> detectedPointVec;
                                    detectedPointVec.push_back(detectedPoint);
                                    std::vector<cv::Point2f> newImgPointVec(1);

                                    /// invert perspective transform to go back to original image coordinates
                                    //cv::Mat invPerspectiveTransf;
                                    //cv::invert(perspectiveTransf, invPerspectiveTransf);
                                    cv::perspectiveTransform(detectedPointVec, newImgPointVec, invPerspectiveTransf);

                                    /// add to centers
                                    centers.push_back(newImgPointVec[0]);
                                    if (m_isAsymmetricGrid)
                                        objectCenters.push_back(cv::Point3f((2*j + i % 2)*m_colWidth, i*m_rowHeight, 0.f));
                                    else
                                        objectCenters.push_back(cv::Point3f(j*m_colWidth, i*m_rowHeight, 0.f));



                                    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                                    int radius = cvRound(circles[i][2]);
                                    // draw the circle center
                                    cv::circle( img, center, 1, cv::Scalar(0,255,0), -1, 8, 0 );
                                    // draw the circle outline
                                    cv::circle( img, center, radius, cv::Scalar(0,0,255), 1, 8, 0 );

                                }
                                cv::imwrite(qPrintable(QString("tmp/debug/imgRectifiedPatternPoint_%1_%2_houghCircle.bmp").arg(j).arg(i)), img);
#endif


#if 0

                                ///
                                int thresh = 100;
                                cv::Mat cannyOutput;
                                std::vector< std::vector< cv::Point > > contours;
                                std::vector< cv::Vec4i > hierarchy;

                                /// Detect edges using canny
                                cv::Canny( imgRectifiedPatternPoint, cannyOutput, m_cannyLowThreshold, m_cannyHighThreshold, m_cannyApertureSize);
                                //cv::Canny( imgRectifiedPatternPoint, cannyOutput, thresh, thresh*2, 3 );
                                /// Find contours
                                cv::findContours( cannyOutput, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

                                /// Draw contours
                                cv::RNG rng(12345);
                                cv::Mat drawing = cv::Mat::zeros( imgRectifiedPatternPoint.size(), CV_8UC3 );
                                cv::cvtColor(imgRectifiedPatternPoint, drawing, cv::COLOR_GRAY2RGB);
                                for( size_t i = 0; i< contours.size(); i++ ) {
                                    cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                                    cv::drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
                                }
                                cv::imwrite(qPrintable(QString("tmp/debug/imgRectifiedPatternPoint_%1_%2_corners.bmp").arg(j).arg(i)), drawing);
#endif


                                /// detect point again
                                std::vector<cv::KeyPoint> keypoints;
                                blobDetector->detect(imgRectifiedPatternPoint, keypoints);
                                cv::Point2f centerPoint(m_refinementImgSize/2, m_refinementImgSize/2);
                                if (keypoints.size()) {
                                    cv::Point2f detectedPoint;
                                    size_t ik;
                                    for (ik = 0; ik < keypoints.size(); ++ik) {
                                        detectedPoint = keypoints[ik].pt;
                                        if (cv::norm(detectedPoint - centerPoint) < 10) {
                                            /// this should be the point we were looking
                                            break;
                                        }
                                    }

                                    if (ik != keypoints.size()) {
                                        /// we found a valid point
                                        std::vector<cv::Point2f> detectedPointVec;
                                        detectedPointVec.push_back(detectedPoint);
                                        std::vector<cv::Point2f> newImgPointVec(1);

                                        /// invert perspective transform to go back to original image coordinates
                                        //cv::Mat invPerspectiveTransf;
                                        //cv::invert(perspectiveTransf, invPerspectiveTransf);
                                        cv::perspectiveTransform(detectedPointVec, newImgPointVec, invPerspectiveTransf);

                                        /// add to centers
                                        centers.push_back(newImgPointVec[0]);
                                        if (m_isAsymmetricGrid)
                                            objectCenters.push_back(cv::Point3f((2*j + i % 2), i, 0.f));
                                        else
                                            objectCenters.push_back(cv::Point3f(j, i, 0.f));

#if defined(DEBUG_PERSPECTIVE_TRANSFORMATION)
                                        const cv::Point2f &newImgPoint = newImgPointVec[0];
                                        const cv::Point2f &originalImgPoint = detectedPatternPoints.at(indices[0]);
                                        if (cv::norm(newImgPoint - originalImgPoint) > 0.05) {
                                            qDebug() << "original:" << originalImgPoint.x << originalImgPoint.y
                                                     << "new:" << newImgPoint.x << newImgPoint.y;
                                        }
                                        cv::circle(imgRectifiedPatternPoint, detectedPoint, 6, cv::Scalar(255));
#endif

                                    } else {
                                        qWarning() << "the detector found" << keypoints.size() << "possible pattern points in the rectified image, but none were correct";
                                    }

                                } else {
                                    qWarning() << "the detector found nothing in the rectified image:";
                                }

#if defined(DEBUG_PERSPECTIVE_TRANSFORMATION)
                                cv::imwrite(qPrintable(QString("tmp/debug/imgRectifiedPatternPoint_%1_%2.bmp").arg(j).arg(i)), imgRectifiedPatternPoint);
#endif

                            } else {
                                /// insert point as is
                                centers.push_back(detectedPatternPoints.at(indices[0]));
                                if (m_isAsymmetricGrid)
                                    objectCenters.push_back(cv::Point3f((2*j + i % 2), i, 0.f));
                                else
                                    objectCenters.push_back(cv::Point3f(j, i, 0.f));
                            }
                        }
                    }
                }

                cv::Mat(centers).copyTo(corners);
                cv::Mat(objectCenters).copyTo(patternPoints);

                return true;
            } else {
                qDebug() << "standard circle grid not found inside rectangle" << iRect;
            }

            iRect++;
        } else {
            qDebug() << "found quadrilateral, but not used because mincos:" << mincos << "maxcos:" << maxcos;
        }
    }

    corners.clear();
    patternPoints.clear();

    qDebug() << "nothing found";
    return false;
}

void ZIncompleteCircleGridPatternFinder::setMaxColumns(int columns)
{
    if (m_completeBoardSize.width != columns) {
        m_completeBoardSize.width = columns;
        emit maxColumnsChanged(columns);
    }
}

void ZIncompleteCircleGridPatternFinder::setMaxRows(int rows)
{
    if (m_completeBoardSize.height != rows) {
        m_completeBoardSize.height = rows;
        emit maxRowsChanged(rows);
    }
}

void ZIncompleteCircleGridPatternFinder::setIsAsymmetricGrid(bool isAsymmetric)
{
    if (m_isAsymmetricGrid != isAsymmetric) {
        m_isAsymmetricGrid = isAsymmetric;
        emit isAsymmetricGridChanged(isAsymmetric);
    }
}

void ZIncompleteCircleGridPatternFinder::setRefinePatternPoints(bool refinePatternPoints)
{
    if (m_refinePatternPoints != refinePatternPoints) {
        m_refinePatternPoints = refinePatternPoints;
        emit refinePatternPointsChanged(refinePatternPoints);
    }
}

void ZIncompleteCircleGridPatternFinder::updateConfigHash()
{
    /// generate unique "hash" for the current configuration
    QString hash = QString("%1|%2|%3|%4|%5")
            .arg(getBaseHash())
            .arg(m_completeBoardSize.width)
            .arg(m_completeBoardSize.height)
            .arg(m_isAsymmetricGrid)
            .arg(m_refinePatternPoints);

    if (m_configHash != hash) {
        m_configHash = hash;
        emit configHashChanged(m_configHash);
    }
}

} // namespace Z3D
