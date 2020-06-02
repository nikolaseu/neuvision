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

#include "zringgrid2patternfinder.h"

#include "ZCore/zlogging.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

//#define DEBUG_BLOB_DETECTOR
//#define DEBUG_REFERENCE_DETECTOR
//#define DEBUG_COORDINATE_SYSTEM
//#define DEBUG_PERSPECTIVE_TRANSFORMATION

#if defined(DEBUG_BLOB_DETECTOR) || defined(DEBUG_REFERENCE_DETECTOR) || defined(DEBUG_COORDINATE_SYSTEM) || defined(DEBUG_PERSPECTIVE_TRANSFORMATION)
#include <QDir>
#include "opencv2/highgui/highgui.hpp" //to use imwrite, imshow
#endif

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcameracalibrator.finders.zringgrid2", QtInfoMsg)

namespace Z3D
{

int getHierarchyLevel(const std::vector<cv::Vec4i> &hierarchies, const int index, const bool traverseSiblings = false) {
    /// hierarchy vector == {Next, Previous, First_Child, Parent}
    const auto &hierarchy = hierarchies[size_t(index)];
    const int next = hierarchy[0];
    const int child = hierarchy[2];

    if (next == -1 && child == -1) {
        // there are no more siblings nor children
        return 1;
    }

    // iterate over same level
    const int siblingsMaxLevel = (traverseSiblings && next != -1)
            ? getHierarchyLevel(hierarchies, next, true)
            : 1;

    // iterate childs
    const int childrensMaxLevel = (child != -1)
            ? 1 + getHierarchyLevel(hierarchies, child, true)
            : 1;

    return std::max(siblingsMaxLevel, childrensMaxLevel);
}


ZRingGrid2PatternFinder::ZRingGrid2PatternFinder(QObject *parent)
    : ZCalibrationPatternFinder(parent)
{
    setColumns(4);
    setRows(4);

    QObject::connect(this, SIGNAL(maxColumnsChanged(int)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(maxRowsChanged(int)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(refinePatternPointsChanged(bool)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(blurSizeChanged(int)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(cannyApertureSizeChanged(int)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(cannyLowThresholdChanged(int)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(cannyHighThresholdChanged(int)),
                     this, SLOT(updateConfigHash()));

    /// generate current config hash
    updateConfigHash();

#if defined(DEBUG_BLOB_DETECTOR) || defined(DEBUG_REFERENCE_DETECTOR) || defined(DEBUG_COORDINATE_SYSTEM) || defined(DEBUG_PERSPECTIVE_TRANSFORMATION)
    /// create folder where the debug images will be saved
    QDir::current().mkpath("tmp/debug");
#endif
}

QString ZRingGrid2PatternFinder::id() const
{
    return QString(metaObject()->className());
}

QString ZRingGrid2PatternFinder::name() const
{
    return QLatin1String("Ring grid (v2)");
}

int ZRingGrid2PatternFinder::maxColumns() const
{
    return m_completeBoardSize.width;
}

int ZRingGrid2PatternFinder::maxRows() const
{
    return m_completeBoardSize.height;
}

bool ZRingGrid2PatternFinder::refinePatternPoints() const
{
    return m_refinePatternPoints;
}

int ZRingGrid2PatternFinder::blurSize() const
{
    return m_blurSize;
}

int ZRingGrid2PatternFinder::cannyLowThreshold() const
{
    return m_cannyLowThreshold;
}

int ZRingGrid2PatternFinder::cannyHighThreshold() const
{
    return m_cannyHighThreshold;
}

int ZRingGrid2PatternFinder::cannyApertureSize() const
{
    return m_cannyApertureSize;
}

bool ZRingGrid2PatternFinder::findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &patternPoints)
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
        zWarning() << "unknown image type";
        return false;
    }

    if (m_equalizeHistogram) {
        /// normalize image
        cv::equalizeHist(gray, gray);
    }

    /// find edges
    /// blur image a little to avoid getting noisy edges
    cv::Mat blurredGray;
    //cv::blur(gray, blurredGray, cv::Size(m_blurSize, m_blurSize));
    cv::medianBlur(gray, blurredGray, m_blurSize);
    cv::Mat bw;
    cv::Canny(blurredGray, bw, m_cannyLowThreshold, m_cannyHighThreshold, m_cannyApertureSize);

#if defined(DEBUG_BLOB_DETECTOR) || defined(DEBUG_REFERENCE_DETECTOR)
    static QAtomicInt atomicInt = 0;
    const int currentIndex = atomicInt.fetchAndAddAcquire(1);
    cv::Mat contoursImage;

    cv::imwrite(qPrintable(QString("tmp/debug/%1_gray.jpg").arg(currentIndex)), gray);
    cv::imwrite(qPrintable(QString("tmp/debug/%1_grayBlurred.jpg").arg(currentIndex)), blurredGray);
    cv::imwrite(qPrintable(QString("tmp/debug/%1_gray_bw.png").arg(currentIndex)), bw != 0);
#endif

    /// find contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchies;
    cv::findContours(bw, contours, hierarchies, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

#if defined(DEBUG_BLOB_DETECTOR)
    cv::cvtColor(image, contoursImage, cv::COLOR_GRAY2RGB);
    for (int level = 6; level > 0; --level) {
        const cv::Scalar color = [level]() -> cv::Scalar {
                switch(level) {
                    case 1: return cv::Scalar(  0,  0,255);
                    case 2: return cv::Scalar(  0,255,  0);
                    case 3: return cv::Scalar(255,  0,  0);
                    case 4: return cv::Scalar(255,255,  0);
                    case 5: return cv::Scalar(255,  0,255);
                    default: return cv::Scalar(0, 255,255);
                }
        }();
        cv::drawContours(contoursImage, contours, -1, color, 1, 8, hierarchies, level);
    }
    cv::imwrite(qPrintable(QString("tmp/debug/%1_contours.png").arg(currentIndex)), contoursImage);

    for (int level = 0; level < 6; ++level) {
        cv::Mat contoursLevelImage;
        cv::cvtColor(image, contoursLevelImage, cv::COLOR_GRAY2RGB);
        const cv::Scalar color = cv::Scalar(0, 0, 255);
        cv::drawContours(contoursLevelImage, contours, -1, color, 1, cv::LINE_8, hierarchies, level);
        cv::imwrite(qPrintable(QString("tmp/debug/%1_contours_%2.png").arg(currentIndex).arg(level)), contoursLevelImage);
    }
#endif

    /// find the first 3 that has at least N inner contours in the hierarchy and satisfies concentric circles condition
    std::vector<size_t> validContourIndexes;
    {
        int index = 0;
        while (index != -1) {
            /// traverse top level hierarchies
            const auto uindex = size_t(index);
            const auto &contour = contours[uindex];
            const auto contourArea = cv::contourArea(contour, false);
            if (contourArea > 256) { // 16x16 px minimum
                /// opencv gives us two contours for each border (inner and outer) since we applied an edge detector first
                /// so three concentric circles are in reality 6 contours
                if (6 <= getHierarchyLevel(hierarchies, index)) {
                    /// check inner contours to check if they are concentric circles
                    cv::Point2f center;
                    float radius;
                    cv::minEnclosingCircle(contour, center, radius);
                    const auto circleArea = M_PI * double(radius * radius);
                    if (std::abs((circleArea / contourArea) - 1.) < 0.25) { // 75% approx
                        validContourIndexes.push_back(uindex);
                    } else {
                        zDebug() << "circleArea:" << circleArea << "contourArea:" << contourArea << "ratio:" << circleArea / contourArea;
                    }
                }
            }

            if (validContourIndexes.size() == 3) {
                break;
            }

            /// hierarchy vector == {Next, Previous, First_Child, Parent}
            index = hierarchies[uindex][0]; // next at same level
        }
    }

#if defined(DEBUG_REFERENCE_DETECTOR)
    cv::cvtColor(image, contoursImage, cv::COLOR_GRAY2RGB);
    for (const auto i : validContourIndexes) {
        cv::drawContours(contoursImage, contours, int(i), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }
    cv::imwrite(qPrintable(QString("tmp/debug/%1_contours_valid.png").arg(currentIndex)), contoursImage);
#endif

    if (validContourIndexes.size() < 3) {
        zDebug() << "could not find coordinate system (special marker rings)";
        return false;
    }

    std::vector<cv::Point> allContours;
    for (const auto i : validContourIndexes) {
        const auto &contour = contours[i];
        allContours.insert(allContours.end(), contour.begin(), contour.end());
    }
    /// get the minimum rect containing the detected rings
    cv::Rect rect = cv::boundingRect(allContours);

    const auto &offset = std::max(rect.height, rect.width) * 0.5;
    rect.x -= offset;
    rect.y -= offset;
    rect.width += offset * 2.;
    rect.height += offset * 2.;

    /// try to find standard circle grid inside rectangle
    /// use only the section of the image that matters
    cv::Mat roi = gray(rect);
    cv::Mat bwRoi = bw(rect);

    /// and only use external contours
    std::vector<std::vector<cv::Point>> roiContours;
    cv::findContours(bwRoi, roiContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat contoursRoi = 255 - roi;
    std::vector<std::vector<cv::Point>> roiContoursFiltered;
    std::copy_if(roiContours.begin(), roiContours.end(), std::back_inserter(roiContoursFiltered), [](const auto &contour) -> bool {
        return cv::contourArea(contour, false) > 100; // 10x10 px minimum size
    });
    cv::drawContours(contoursRoi, roiContoursFiltered, -1, cv::Scalar(0), cv::FILLED, cv::LINE_AA);
    cv::blur(contoursRoi, contoursRoi, cv::Size(3, 3));
    std::vector<cv::KeyPoint> keypoints;

    cv::SimpleBlobDetector::Params blobDetectorParams;
    blobDetectorParams.filterByCircularity = true;
    blobDetectorParams.minCircularity = 0.85f; // 1 == perfect circle, 0.785 == square
    blobDetectorParams.maxCircularity = 1.f;
    blobDetectorParams.filterByArea = true;
    blobDetectorParams.minArea = 100; // 10x10 px
    blobDetectorParams.maxArea = 10000; // 100x100 px

#if defined(CV_VERSION_EPOCH) && CV_VERSION_EPOCH < 3 // OpenCV 2
    cv::Ptr<cv::FeatureDetector> blobDetector( new cv::SimpleBlobDetector(blobDetectorParams) );
#else // OpenCV 3
    cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(blobDetectorParams);
#endif

#if defined(DEBUG_REFERENCE_DETECTOR)
    cv::imwrite(qPrintable(QString("tmp/debug/%1_blobDetectorRoi.png").arg(currentIndex)), roi);
    cv::imwrite(qPrintable(QString("tmp/debug/%1_blobDetectorRoiContours.png").arg(currentIndex)), contoursRoi);

    /// test blobDetector
    blobDetector->detect(contoursRoi, keypoints);
    cv::Mat roiMaskKeypoints;
    cv::cvtColor(roi, roiMaskKeypoints, cv::COLOR_GRAY2RGB);
    for (const auto &keypoint : keypoints) {
        cv::circle(roiMaskKeypoints, keypoint.pt, 3, cv::Scalar(0,0,255));
    }
    cv::imwrite(qPrintable(QString("tmp/debug/%1_blobDetectorRoiKeypoints.png").arg(currentIndex)), roiMaskKeypoints);
#endif

    if (!cv::findCirclesGrid(contoursRoi, m_boardSize, corners,
                            // cv::CALIB_CB_CLUSTERING | // this doesn't work sometimes
                            cv::CALIB_CB_SYMMETRIC_GRID,
                            blobDetector)) {
        zDebug() << "standard circle grid not found near bigger rings";
        return false;
    }

#if defined(DEBUG_REFERENCE_DETECTOR)
    cv::Mat circlesGridImage;
    cv::cvtColor(roi, circlesGridImage, cv::COLOR_GRAY2RGB);
    cv::drawChessboardCorners(circlesGridImage, m_boardSize, corners, true);
    cv::imwrite(qPrintable(QString("tmp/debug/%1_blobDetectorCircleGrid.png").arg(currentIndex)), circlesGridImage);
#endif

    /// the standard circle grid was found
    /// return corners to original image coordinates
    const cv::Point2f origin(rect.x, rect.y);
    for (auto c = corners.begin(); c != corners.end(); ++c) {
        *c += origin;
    }

    /// get coordinates of ideal points, the indices inside the standard grid
    std::vector<cv::Point> idealPoints;
    /// generate pattern object coordinates
    for (int i=0; i<m_boardSize.height; ++i) {
        for (int j=0; j<m_boardSize.width; ++j) {
            idealPoints.push_back(cv::Point(j, i));
        }
    }

    /// find markers correspondences to stablish reference coord system
    int cornerIndexes[3] = { -1, -1, -1 };
    for (size_t index = 0; index < 3; ++index) {
        const auto i = validContourIndexes[index];
        const auto &contour = contours[i];
        int cIndex = 0;
        for (auto c = corners.cbegin(); c != corners.cend(); ++c, ++cIndex) {
            if (cv::pointPolygonTest(contour, *c, false) > 0) {
                cornerIndexes[index] = cIndex;
                break;
            }
        }
    }

    /// only continue if coord system was found
    for (const auto i : cornerIndexes) {
        if (i < 0) {
            zDebug() << "could not find reference coord system";
            return false;
        }
    }

    /// xmap will map (x coord) -> (point count on that x coord)
    /// ymap will map (y coord) -> (point count on that y coord)
    std::map<int, int> xmap;
    std::map<int, int> ymap;
    for (const auto i : cornerIndexes) {
        const auto &patternPoint = idealPoints[size_t(i)]; // safe because we already checked if >= 0

        if (xmap.find(patternPoint.x) == xmap.end()) {
            xmap[patternPoint.x] = 1;
        } else {
            xmap[patternPoint.x] += 1;
        }

        if (ymap.find(patternPoint.y) == ymap.end()) {
            ymap[patternPoint.y] = 1;
        } else {
            ymap[patternPoint.y] += 1;
        }

#if defined(DEBUG_COORDINATE_SYSTEM)
        const auto &imagePoint = corners[size_t(i)];
        zDebug() << QString("pattern coords: (%1,%2) => (%3,%4)")
                    .arg(double(patternPoint.x))
                    .arg(double(patternPoint.y))
                    .arg(double(imagePoint.x))
                    .arg(double(imagePoint.y));
#endif
    }

    /// I know that for the y axis, the x coord will be repeated 2 times
    int yAxisCol = -1;
    for (auto it = xmap.cbegin(); it != xmap.cend(); ++it) {
        if (it->second > 1) {
            yAxisCol = it->first;
            break;
        }
    }

    if (yAxisCol < 0) {
        /// TODO
        zDebug() << "y axis not found, pattern is rotated?";
        return false;
    }

    int sumY = 0;
    for (const auto i : cornerIndexes) {
        const auto &patternPoint = idealPoints[size_t(i)]; // safe because we already checked if >= 0
        sumY += patternPoint.y;
    }
    const int xAxisRow = sumY / 3;

    int originIndex = -1, xIndex = -1, yIndex = -1;
    cv::Point2f originPattern, xPattern, yPattern;
    for (size_t i = 0; i < idealPoints.size() && (originIndex < 0 || xIndex < 0 || yIndex < 0); ++i) {
        const auto &patternPoint = idealPoints[i];
        if ((originIndex < 0) && (patternPoint.x == yAxisCol) && (patternPoint.y == xAxisRow)) {
            originIndex = i;
            originPattern = patternPoint;
        } else if ((xIndex < 0) && (patternPoint.x - yAxisCol == 1) && (patternPoint.y == xAxisRow)) {
            xIndex = i;
            xPattern = patternPoint;
        } else if ((yIndex < 0) && (patternPoint.x == yAxisCol) && (patternPoint.y - xAxisRow == 1)) {
            yIndex = i;
            yPattern = patternPoint;
        }
    }

    if (originIndex < 0 || xIndex < 0 || yIndex < 0) {
        zWarning() << "we couldn't match markers to coord system!";
        return false;
    }

#if defined(DEBUG_COORDINATE_SYSTEM)
    cv::Mat coordSystemDebugImg;
    cv::cvtColor(gray, coordSystemDebugImg, cv::COLOR_GRAY2RGB);
    cv::drawMarker(coordSystemDebugImg, corners[originIndex], cv::Scalar(0,0,255), cv::MarkerTypes::MARKER_TILTED_CROSS, 15, 2);
    cv::drawMarker(coordSystemDebugImg, corners[xIndex], cv::Scalar(0,0,255), cv::MarkerTypes::MARKER_STAR, 2, 3);
    cv::drawMarker(coordSystemDebugImg, corners[yIndex], cv::Scalar(0,0,255), cv::MarkerTypes::MARKER_STAR, 2, 3);
    cv::imwrite(qPrintable(QString("tmp/debug/%1_gray_coordSystem.png").arg(currentIndex)), coordSystemDebugImg);
#endif

    /// detect all possible pattern points
    /// only use external contours
    std::vector<std::vector<cv::Point>> simpleContours;
    cv::findContours(bw, simpleContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat simpleGray = 255 - gray;
    std::vector<std::vector<cv::Point>> simpleContoursFiltered;
    std::copy_if(simpleContours.begin(), simpleContours.end(), std::back_inserter(simpleContoursFiltered), [](const auto &contour) -> bool {
        return cv::contourArea(contour, false) > 64; // 8x8 px minimum size
    });
    cv::drawContours(simpleGray, simpleContoursFiltered, -1, cv::Scalar(0), cv::FILLED, cv::LINE_AA);
    cv::blur(simpleGray, simpleGray, cv::Size(3, 3));

#if defined(DEBUG_PERSPECTIVE_TRANSFORMATION)
    cv::imwrite(qPrintable(QString("tmp/debug/%1_gray_simplified.png").arg(currentIndex)), simpleGray);
#endif

    blobDetector->detect(simpleGray, keypoints);
    std::vector<cv::Point2f> detectedPatternPoints;
    for (const auto &keypoint : keypoints) {
        detectedPatternPoints.push_back(keypoint.pt);
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
    for (int i2 = 0; i2 < m_completeBoardSize.height; i2++) {
        /// first index always coordinate origin
        if (i2 >= m_boardSize.height+extraRows) {
            i = m_boardSize.height+extraRows - i2 - 1;
        } else {
            i = i2;
        }

        for (int j2 = 0; j2 < m_completeBoardSize.width; j2++) {
            /// first index always coordinate origin
            if (j2 >= m_boardSize.width+extraCols) {
                j = m_boardSize.width+extraCols - j2 - 1;
            } else {
                j = j2;
            }

            cv::Point2f idealPt = cv::Point2f(j, i);

            std::vector<float> query = cv::Mat(idealPt);
            int knn = 1;
            std::vector<int> indices(knn);
            std::vector<float> dists(knn);
            flannIndex.knnSearch(query, indices, dists, knn, cv::flann::SearchParams());

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
                    rectCornerPoints[0] = center + m_refinementZoomArea * cv::Point2f(-1.f, -1.f);
                    /// top right
                    rectCornerPoints[1] = center + m_refinementZoomArea * cv::Point2f(1.f, -1.f);
                    /// bottom right
                    rectCornerPoints[2] = center + m_refinementZoomArea * cv::Point2f(1.f, 1.f);
                    /// bottom left
                    rectCornerPoints[3] = center + m_refinementZoomArea * cv::Point2f(-1.f, 1.f);

                    cv::Mat idealToImgHomography = cv::findHomography(cv::Mat(idealPoints), cv::Mat(corners), 0);
                    cv::Mat imageRectCornersPointsMat;
                    cv::transform(rectCornerPoints, imageRectCornersPointsMat, idealToImgHomography);
                    std::vector<cv::Point2f> imageRectCornersPoints;
                    cv::convertPointsFromHomogeneous(imageRectCornersPointsMat, imageRectCornersPoints);

                    /// destination image
                    static const int m_refinementImgSize = 100;
                    cv::Mat imgRectifiedPatternPoint = cv::Mat::zeros(m_refinementImgSize, m_refinementImgSize, gray.type());

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
                            objectCenters.push_back(cv::Point3f(j, i, 0.f));

#if defined(DEBUG_PERSPECTIVE_TRANSFORMATION)
                            const cv::Point2f &newImgPoint = newImgPointVec[0];
                            const cv::Point2f &originalImgPoint = detectedPatternPoints.at(indices[0]);
                            if (cv::norm(newImgPoint - originalImgPoint) > 0.05) {
                                zDebug() << "original:" << originalImgPoint.x << originalImgPoint.y
                                         << "new:" << newImgPoint.x << newImgPoint.y;
                            }
                            cv::circle(imgRectifiedPatternPoint, detectedPoint, 6, cv::Scalar(255));
#endif

                        } else {
                            zWarning() << "the detector found" << keypoints.size() << "possible pattern points in the rectified image, but none were correct";
                        }

                    } else {
                        zWarning() << "the detector found nothing in the rectified image:";
                    }

#if defined(DEBUG_PERSPECTIVE_TRANSFORMATION)
                    cv::imwrite(qPrintable(QString("tmp/debug/imgRectifiedPatternPoint_%1_%2.png").arg(j).arg(i)), imgRectifiedPatternPoint);
#endif

                } else {
                    /// insert point as is
                    centers.push_back(detectedPatternPoints.at(indices[0]));
                    objectCenters.push_back(cv::Point3f(j, i, 0.f));
                }
            }
        }
    }

    cv::Mat(centers).copyTo(corners);
    cv::Mat(objectCenters).copyTo(patternPoints);

    /// transform to use correct pattern coord system
    xPattern -= originPattern;
    yPattern -= originPattern;

    const bool shouldSwitchAxis = (std::abs(xPattern.y) == 2. && std::abs(yPattern.x) == 1.);
    if (shouldSwitchAxis) {
        std::swap(xPattern, yPattern);
    }

    const bool shouldInvertY = yPattern.y < 0;
    const bool shouldInvertX = xPattern.x < 0;

    for (auto it = patternPoints.begin(); it != patternPoints.end(); ++it) {
        it->x -= originPattern.x;
        it->y -= originPattern.y;

        if (shouldSwitchAxis) {
            std::swap(it->x, it->y);
        }

        if (shouldInvertY) {
            it->y *= -1.;
        }

        if (shouldInvertX) {
            it->x *= -1.;
        }
    }

    return true;
}

void ZRingGrid2PatternFinder::setMaxColumns(int columns)
{
    if (m_completeBoardSize.width != columns) {
        m_completeBoardSize.width = columns;
        emit maxColumnsChanged(columns);
    }
}

void ZRingGrid2PatternFinder::setMaxRows(int rows)
{
    if (m_completeBoardSize.height != rows) {
        m_completeBoardSize.height = rows;
        emit maxRowsChanged(rows);
    }
}

void ZRingGrid2PatternFinder::setRefinePatternPoints(bool refinePatternPoints)
{
    if (m_refinePatternPoints == refinePatternPoints)
        return;

    m_refinePatternPoints = refinePatternPoints;
    emit refinePatternPointsChanged(m_refinePatternPoints);
}

void ZRingGrid2PatternFinder::setBlurSize(int blurSize)
{
    if (m_blurSize == blurSize)
        return;

    m_blurSize = blurSize;
    emit blurSizeChanged(m_blurSize);
}

void ZRingGrid2PatternFinder::setCannyLowThreshold(int cannyLowThreshold)
{
    if (m_cannyLowThreshold == cannyLowThreshold)
        return;

    m_cannyLowThreshold = cannyLowThreshold;
    emit cannyLowThresholdChanged(m_cannyLowThreshold);
}

void ZRingGrid2PatternFinder::setCannyHighThreshold(int cannyHighThreshold)
{
    if (m_cannyHighThreshold == cannyHighThreshold)
        return;

    m_cannyHighThreshold = cannyHighThreshold;
    emit cannyHighThresholdChanged(m_cannyHighThreshold);
}

void ZRingGrid2PatternFinder::setCannyApertureSize(int cannyApertureSize)
{
    if (m_cannyApertureSize == cannyApertureSize)
        return;

    m_cannyApertureSize = cannyApertureSize;
    emit cannyApertureSizeChanged(m_cannyApertureSize);
}

void ZRingGrid2PatternFinder::updateConfigHash()
{
    /// generate unique "hash" for the current configuration
    QString hash = QString("%1|%2|%3|%4|%5|%6|%7|%8")
            .arg(getBaseHash())
            .arg(m_completeBoardSize.width)
            .arg(m_completeBoardSize.height)
            .arg(m_refinePatternPoints)
            .arg(m_blurSize)
            .arg(m_cannyLowThreshold)
            .arg(m_cannyHighThreshold)
            .arg(m_cannyApertureSize);

    if (m_configHash != hash) {
        m_configHash = hash;
        emit configHashChanged(m_configHash);
    }
}

} // namespace Z3D
