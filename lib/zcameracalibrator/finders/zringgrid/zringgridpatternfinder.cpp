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

#include "zringgridpatternfinder.h"

#include "ZCore/zlogging.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

//#define DEBUG_REFERENCE_DETECTOR
//#define DEBUG_PERSPECTIVE_TRANSFORMATION

#if defined(DEBUG_REFERENCE_DETECTOR) || defined(DEBUG_PERSPECTIVE_TRANSFORMATION)
#  include <QDir>
#  include "opencv2/highgui.hpp" //to use imwrite, imshow
#endif

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcameracalibrator.finders.zringgrid2", QtInfoMsg)

namespace Z3D
{

bool  m_equalizeHistogram = (false);
int   m_cannyLowThreshold = (0);
int   m_cannyHighThreshold = (50);
int   m_cannyApertureSize = (3);
float m_squareSize = (1.0f);
float m_maxRectifiedDistance = (m_squareSize / 2.f);


ZRingGridPatternFinder::ZRingGridPatternFinder(QObject *parent)
    : ZCalibrationPatternFinder(parent)
    , m_completeBoardSize(25, 25)
    , m_refinePatternPoints(false)
{
    setColumns(4);
    setRows(4);

    QObject::connect(this, &ZRingGridPatternFinder::maxColumnsChanged,
                     this, &ZRingGridPatternFinder::updateConfigHash);
    QObject::connect(this, &ZRingGridPatternFinder::maxRowsChanged,
                     this, &ZRingGridPatternFinder::updateConfigHash);
    QObject::connect(this, &ZRingGridPatternFinder::refinePatternPointsChanged,
                     this, &ZRingGridPatternFinder::updateConfigHash);

    /// generate current config hash
    updateConfigHash();

#if defined(DEBUG_RECTANGLE_DETECTOR) || defined(DEBUG_PERSPECTIVE_TRANSFORMATION)
    /// create folder where the debug images will be saved
    QDir::current().mkpath("tmp/debug");
#endif
}

QString ZRingGridPatternFinder::id() const
{
    return QString(metaObject()->className());
}

QString ZRingGridPatternFinder::name() const
{
    return QLatin1String("Ring grid");
}

int ZRingGridPatternFinder::maxColumns() const
{
    return m_completeBoardSize.width;
}

int ZRingGridPatternFinder::maxRows() const
{
    return m_completeBoardSize.height;
}

bool ZRingGridPatternFinder::refinePatternPoints() const
{
    return m_refinePatternPoints;
}

bool ZRingGridPatternFinder::findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &patternPoints)
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
    cv::blur(gray, blurredGray, cv::Size(3,3));
    cv::Mat bw;
    cv::Canny(blurredGray, bw, m_cannyLowThreshold, m_cannyHighThreshold, m_cannyApertureSize);

    /// find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(bw, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    /// sort by contour size
    std::sort(contours.begin(), contours.end(), [](const auto &a, const auto &b) -> bool {
        return cv::contourArea(a, false) > cv::contourArea(b, false);
    });

#ifdef DEBUG_REFERENCE_DETECTOR
    static QAtomicInt atomicInt = 0;
    const int currentIndex = atomicInt.fetchAndAddAcquire(1);
    //cv::imwrite(qPrintable(QString("%1gray.jpg").arg(tmpint)), gray);
    cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_bw.bmp").arg(currentIndex)), bw!=0);
    cv::Mat contoursImage;

    cv::cvtColor(image, contoursImage, cv::COLOR_GRAY2RGB);
    for (size_t i = 0; i < contours.size(); i++) {
        cv::drawContours(contoursImage, contours, i, cv::Scalar(0,0,255), 2);
    }
    cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_contours.bmp").arg(currentIndex)), contoursImage);

    /// show 3 bigger ones
    cv::cvtColor(image, contoursImage, cv::COLOR_GRAY2RGB);
    for (size_t i = 0; i < 3; i++) {
        cv::drawContours(contoursImage, contours, i, cv::Scalar(0,0,255), 5);
    }
    cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_contours_valid.bmp").arg(currentIndex)), contoursImage);
#endif

    std::vector<cv::Point> allContours;
    for (size_t i = 0; i < 3; i++) {
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

    cv::SimpleBlobDetector::Params blobDetectorParams;
    blobDetectorParams.filterByArea = false;

    std::vector<cv::KeyPoint> keypoints;

    cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create();

#ifdef DEBUG_REFERENCE_DETECTOR
    cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_roi.bmp").arg(currentIndex)), roi);

    /// test blobDetector
    cv::Mat roiMaskKeypoints = roi.clone();
    blobDetector->detect(roi, keypoints);
    for (size_t i = 0; i < keypoints.size(); i++) {
        cv::circle(roiMaskKeypoints, keypoints[i].pt, 3, cv::Scalar(255));
    }
    cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_roiKeypoints.bmp").arg(currentIndex)), roiMaskKeypoints);
#endif

    if (!cv::findCirclesGrid(roi, m_boardSize, corners,
                            //cv::CALIB_CB_CLUSTERING |
                            cv::CALIB_CB_SYMMETRIC_GRID,
                            blobDetector)) {
        zDebug() << "standard circle grid not found near bigger rings";
        return false;
    }

    /// the standard circle grid was found
    /// return corners to original image coordinates
    cv::Point2f origin(rect.x, rect.y);
    for (std::vector<cv::Point2f>::iterator c = corners.begin(); c != corners.end(); ++c)
        *c += origin;

    /// get coordinates of ideal points, the indices inside the standard grid
    std::vector<cv::Point2f> idealPoints;
    /// generate pattern object coordinates
    for (int i=0; i<m_boardSize.height; ++i) {
        for (int j=0; j<m_boardSize.width; ++j) {
            idealPoints.push_back(cv::Point2f(j, i));
        }
    }

    /// find markers correspondences to stablish reference coord system
    int cornerIndexes[3] = {-1, -1, -1};
    for (size_t i = 0; i < 3; i++) {
        const auto &contour = contours[i];
        int cIndex = 0;
        for (auto c = corners.cbegin(); c != corners.cend(); ++c, ++cIndex) {
            if (cv::pointPolygonTest(contour, *c, false) > 0) {
                cornerIndexes[i] = cIndex;
                break;
            }
        }
    }

    bool foundCoordSystem = true;
    for (size_t i = 0; i < 3; i++) {
        if (cornerIndexes[i] < 0) {
            foundCoordSystem = false;
            break;
        }
    }

    /// only continue if coord system was found
    if (!foundCoordSystem) {
        zDebug() << "could not find reference coord system";
        return false;
    }

    std::map<float, int> xmap;
    std::map<float, int> ymap;
    for (size_t i = 0; i < 3; i++) {
        auto index = size_t(cornerIndexes[i]); // safe because we already checked if >= 0
        const auto &patternPoint = idealPoints[index];

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

#ifdef DEBUG_REFERENCE_DETECTOR
        const auto &imagePoint = corners[index];
        zDebug() << QString("pattern coords: (%1,%2) => (%3,%4)")
                    .arg(double(patternPoint.x))
                    .arg(double(patternPoint.y))
                    .arg(double(imagePoint.x))
                    .arg(double(imagePoint.y));
#endif
    }

    float originX = -999.0, originY = -999.0; // if something goes wrong, we'll notice
    for (auto it = xmap.cbegin(); it != xmap.cend(); ++it) {
        if (it->second > 1) {
            originX = it->first;
            break;
        }
    }

    for (auto it = ymap.cbegin(); it != ymap.cend(); ++it) {
        if (it->second > 1) {
            originY = it->first;
            break;
        }
    }

    int originIndex = -1, xIndex = -1, yIndex = -1;
    cv::Point2f originPattern, xPattern, yPattern;
    for (size_t i = 0; i < 3; i++) {
        auto index = size_t(cornerIndexes[i]); // safe because we already checked if >= 0
        const auto &patternPoint = idealPoints[index];
        if (originIndex < 0 && patternPoint.x == originX && patternPoint.y == originY) {
            originIndex = index;
            originPattern = patternPoint;
        } else if (xIndex < 0 && (std::fabs(patternPoint.x - originX) == 2.f || std::fabs(patternPoint.y - originY) == 2.f)) {
            xIndex = index;
            xPattern = patternPoint;
        } else if (yIndex < 0 && (std::fabs(patternPoint.x - originX) == 1.f || std::fabs(patternPoint.y - originY) == 1.f)) {
            yIndex = index;
            yPattern = patternPoint;
        } else {
            zWarning() << "we couldn't match markers to coord system!";
            return false;
        }
    }

#ifdef DEBUG_REFERENCE_DETECTOR
    cv::Mat coordSystemDebugImg = gray.clone();
    cv::drawMarker(coordSystemDebugImg, corners[originIndex], cv::Scalar(255, 0, 0), cv::MarkerTypes::MARKER_CROSS);
    cv::circle(coordSystemDebugImg, corners[xIndex], 1, cv::Scalar(255, 0, 0));
    cv::circle(coordSystemDebugImg, corners[yIndex], 1, cv::Scalar(255, 0, 0));
    cv::imwrite(qPrintable(QString("tmp/debug/rectDetector_%1_coordSystem.bmp").arg(currentIndex)), coordSystemDebugImg);
#endif

    /// detect all possible pattern points
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
                    cv::imwrite(qPrintable(QString("tmp/debug/imgRectifiedPatternPoint_%1_%2.bmp").arg(j).arg(i)), imgRectifiedPatternPoint);
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

void ZRingGridPatternFinder::setMaxColumns(int columns)
{
    if (m_completeBoardSize.width != columns) {
        m_completeBoardSize.width = columns;
        emit maxColumnsChanged(columns);
    }
}

void ZRingGridPatternFinder::setMaxRows(int rows)
{
    if (m_completeBoardSize.height != rows) {
        m_completeBoardSize.height = rows;
        emit maxRowsChanged(rows);
    }
}

void ZRingGridPatternFinder::setRefinePatternPoints(bool refinePatternPoints)
{
    if (m_refinePatternPoints != refinePatternPoints) {
        m_refinePatternPoints = refinePatternPoints;
        emit refinePatternPointsChanged(refinePatternPoints);
    }
}

void ZRingGridPatternFinder::updateConfigHash()
{
    /// generate unique "hash" for the current configuration
    QString hash = QString("%1|%2|%3|%4")
            .arg(getBaseHash())
            .arg(m_completeBoardSize.width)
            .arg(m_completeBoardSize.height)
            .arg(m_refinePatternPoints);

    if (m_configHash != hash) {
        m_configHash = hash;
        emit configHashChanged(m_configHash);
    }
}

} // namespace Z3D
