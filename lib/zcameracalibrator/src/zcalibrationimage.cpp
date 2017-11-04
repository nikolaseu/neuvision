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

#include "zcalibrationimage.h"

#include "zcalibrationpatternfinder.h"
#include "zcameracalibration.h"

#include <QDebug>
#include <QFile>
#include <QPainter>

#include <opencv2/imgcodecs.hpp> // imread

#ifndef MAX_THUMBNAIL_SIZE
#define MAX_THUMBNAIL_SIZE 218
#endif

namespace Z3D
{

//! FIXME esto no va acá
static int _sharedPointerMetaTypeId = qRegisterMetaType< Z3D::ZCalibrationImagePtr >("Z3D::ZCalibrationImagePtr");
static int _imageStateMetaTypeId = qRegisterMetaType< Z3D::ZCalibrationImage::ImageState >("Z3D::ZCalibrationImage::ImageState");

ZCalibrationImage::ZCalibrationImage(QString fileName, QObject *parent)
    : QObject(parent)
    , m_fileName(fileName)
    , m_width(-1)
    , m_height(-1)
    , m_state(UnknownState)
{
    //qDebug() << "created image:" << m_fileName;
}

ZCalibrationImage::~ZCalibrationImage()
{
    //qDebug() << "destroying image:" << m_fileName;
}

bool ZCalibrationImage::isValid()
{
    /// image is valid if we could load basic info about it
    if (m_state == UnknownState)
        loadBasicImageInfo();

    return m_state != InvalidState;
}

QString ZCalibrationImage::fileName() const
{
    return m_fileName;
}

int ZCalibrationImage::width() const
{
    return m_width;
}

int ZCalibrationImage::height() const
{
    return m_height;
}

QImage ZCalibrationImage::imageFromUrl(int viewStyle) const
{
    QImage image;

    switch (viewStyle) {
    case OriginalImage:
        image = QImage(m_fileName);
        break;
    case ThumbnailImage:
        if (state() & CheckedState) {
            image = m_patternThumbnail;
        } else {
            image = m_thumbnail;
        }
        break;
    case PatternImage:
        if (state() == PatternFoundState) {
            image = QImage(m_fileName);

            /// Drawing into a QImage with QImage::Format_Indexed8 is not supported
            if (image.format() == QImage::Format_Indexed8)
                image = image.convertToFormat(QImage::Format_ARGB32_Premultiplied);

            /// set up painter to draw over the original image
            QPainter painter(&image);
            //painter.setRenderHint(QPainter::Antialiasing);

            /// qpainter seems to start in 1 instead of 0?
            /// without the 1px offset all marks appear with a bias to the top left
            const int offset = 0;

            if (true) {
                const cv::Point2f &point = m_corners[0];
                const cv::Point2f &nextPoint = m_corners[1];
                cv::Point2f dir = nextPoint - point;
                float normDir = cv::norm(dir);
                dir = dir * (1.f/normDir);
                int markerSize = std::max(normDir / 10.f, 5.f);

                painter.setPen(QPen(QBrush(Qt::red), 2));
                const cv::Point2f startPoint = point + dir*(markerSize+1);
                const cv::Point2f endPoint = nextPoint - dir*(normDir/2.f);
                painter.drawLine(offset+startPoint.x, offset+startPoint.y, offset+endPoint.x, offset+endPoint.y);

                /// draw corners as green X
                for (unsigned int i=0; i<m_corners.size(); ++i) {
                    const cv::Point2f &point = m_corners[i];
                    if (i==0) {
                        painter.setPen(QPen(QBrush(Qt::red), 2));
                        painter.drawEllipse(offset+point.x-markerSize, offset+point.y-markerSize,
                                            2*markerSize+1, 2*markerSize+1);
                    } else {
                        painter.setPen(QPen(QBrush(Qt::green), 2));
                        painter.drawLine(offset+point.x-markerSize/2, offset+point.y-markerSize/2,
                                         offset+point.x+markerSize/2, offset+point.y+markerSize/2);
                        painter.drawLine(offset+point.x-markerSize/2, offset+point.y+markerSize/2,
                                         offset+point.x+markerSize/2, offset+point.y-markerSize/2);
                    }
                }
            } else {
                /// draw corners as green circles
                const cv::Point2f &point = m_corners[0];
                const cv::Point2f &nextPoint = m_corners[1];
                cv::Point2f dir = nextPoint - point;
                float normDir = cv::norm(dir);
                dir = dir * (1.f/normDir);
                int markerSize = normDir / 3.f;

                ///
                painter.setPen(QPen(QBrush(Qt::red), 2));

                painter.drawEllipse(offset+point.x-markerSize/2, offset+point.y-markerSize/2,
                                    markerSize, markerSize);

                const cv::Point2f startPoint = point + dir*(markerSize/2);
                const cv::Point2f endPoint = nextPoint - dir*(markerSize/2);
                painter.drawLine(startPoint.x, startPoint.y, endPoint.x, endPoint.y);

                painter.setPen(QPen(QBrush(Qt::green), 2));
                for (unsigned int i=1; i<m_corners.size(); ++i) {
                    const cv::Point2f &point = m_corners[i];
                    painter.drawEllipse(offset+point.x-markerSize/2, offset+point.y-markerSize/2,
                                        markerSize, markerSize);
                }
            }
        } else {
            //qDebug() << "requested pattern image but the pattern wasn't detected. returning original image...";
            image = QImage(m_fileName);
        }
        break; // useless
    default:
        qWarning() << "invalid viewStyle requested" << viewStyle;
    }

    return image;
}

QPixmap ZCalibrationImage::pixmapFromUrl(int viewStyle) const
{
    return QPixmap::fromImage( imageFromUrl(viewStyle) );
}

QImage ZCalibrationImage::thumbnail() const
{
    return m_thumbnail;
}

cv::Mat ZCalibrationImage::mat() const
{
    cv::Mat img = cv::imread(qPrintable(m_fileName), cv::IMREAD_GRAYSCALE);
    return img;
}

std::vector<cv::Point2f> ZCalibrationImage::detectedPoints() const
{
    return m_corners;
}

std::vector<cv::Point3f> ZCalibrationImage::detectedPointsInPatternCoordinates() const
{
    return m_patternCorners;
}

std::vector<cv::Point3f> ZCalibrationImage::detectedPointsInRealCoordinates() const
{
    return m_worldCorners;
}

cv::Matx33d ZCalibrationImage::rotation() const
{
    return m_rotation;
}

cv::Point3d ZCalibrationImage::translation() const
{
    return m_translation;
}

Z3D::ZCalibrationImage::ImageState ZCalibrationImage::state() const
{
    return m_state;
}

bool ZCalibrationImage::findPattern(ZCalibrationPatternFinder *patternFinder)
{
    QString patternFinderHash = patternFinder->configHash();
    if (m_patternFinderHash != patternFinderHash) {
        m_patternFinderHash = patternFinderHash;

        setState(CheckingState);

        bool patternFound;

        try {
            patternFound = patternFinder->findCalibrationPattern(mat(), m_corners, m_worldCorners, m_patternCorners);
        } catch (std::exception e) {
            qWarning() << "exception thrown while finding calibration pattern:" << e.what();
            patternFound = false;
        }

        /// check if we need to update pattern thumbnail or restore from default
        if (patternFound) {
            /// update pattern thumbnail
            m_patternThumbnail = m_thumbnail;

            /// scale we need to use to transform the corner coordinates for the thumbnail
            float scale = float(m_patternThumbnail.width()) / float(m_width);

            /// draw pattern points
            QPainter painter(&m_patternThumbnail);
            painter.setRenderHint(QPainter::Antialiasing);

            painter.setPen(QPen(QBrush(Qt::green), 3));
            for (unsigned int i=0; i<m_corners.size(); ++i) {
                const cv::Point2f &point = m_corners[i] * scale;
                painter.drawPoint(point.x, point.y);
            }

            int index = -1;
            for (auto patternPoint : m_patternCorners) {
                index++;
                if (patternPoint.x == 0 && patternPoint.y == 0) {
                    const cv::Point2f &point = m_corners[index] * scale;
                    painter.setPen(QPen(QBrush(Qt::red), 3));
                    painter.drawPoint(point.x, point.y);
                    break;
                }
            }

            QFile cornersFile(QString("%1.corners.txt").arg(m_fileName));
            if (cornersFile.open(QFile::WriteOnly | QFile::Truncate | QFile::Text)) {
                QTextStream textStream(&cornersFile);
                textStream << "ximage;yimage;xreal;yreal;zreal\n"
                           << "-------------------------------\n";
                for (size_t i = 0; i<m_corners.size(); ++i) {
                    const cv::Point2f &point = m_corners[i];
                    const cv::Point3f &realPoint = m_worldCorners[i];
                    textStream << point.x << ";" << point.y << ";"
                               << realPoint.x << ";" << realPoint.y << ";" << realPoint.z
                               << '\n';
                }
                cornersFile.close();
            }
        } else {
            m_corners.clear();
            m_worldCorners.clear();
            m_patternCorners.clear();

            /// create "pattern not found" thumbnail
            m_patternThumbnail = m_thumbnail;
            QPainter painter(&m_patternThumbnail);
            painter.setRenderHint(QPainter::Antialiasing);
            painter.setPen(QPen(QBrush(Qt::red), 3));
            /// draw red cross in the top left corner
            painter.drawLine(10, 10, 30, 30);
            painter.drawLine(10, 30, 30, 10);
        }

        if (patternFound) {
            setState(PatternFoundState);
        } else {
            setState(CheckedState);
        }
    }

    return m_state == PatternFoundState;
}

bool ZCalibrationImage::estimateCalibrationPatternPosition(ZCameraCalibration *cameraCalibration)
{
    return cameraCalibration->getEstimatedTransformation(m_corners, m_worldCorners, m_rotation, m_translation);
}

bool ZCalibrationImage::loadBasicImageInfo()
{
    QImage fullImage = QImage(m_fileName);
    if (fullImage.isNull()) {
        qWarning() << "image is null" << m_fileName;
        setState(InvalidState);
        return false;
    }

    m_width = fullImage.width();
    m_height = fullImage.height();
    /// always scale to width for now... it looks better in the list view
    //if (m_width > m_height)
        m_thumbnail = fullImage.scaledToWidth(MAX_THUMBNAIL_SIZE, Qt::SmoothTransformation);
    //else
    //    m_thumbnail = fullImage.scaledToHeight(MAX_THUMBNAIL_SIZE, Qt::SmoothTransformation);

    setState(UncheckedState);

    /// everything is fine if we reach the end
    return true;
}

void ZCalibrationImage::setState(Z3D::ZCalibrationImage::ImageState arg)
{
    if (m_state != arg) {
        m_state = arg;
        emit stateChanged(arg);
    }
}

} // namespace Z3D
