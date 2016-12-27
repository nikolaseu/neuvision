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

#include "zltscalibrationimage.h"

#include <QDebug>
#include <QPainter>

#include "opencv2/imgproc/imgproc.hpp"

#include "stdio.h" // fopen, fscanf

namespace Z3D
{

static int _sharedPointerMetaTypeId = qRegisterMetaType< Z3D::ZLTSCalibrationImage::Ptr >("Z3D::ZLTSCalibrationImage::Ptr");
static int _imageStateMetaTypeId = qRegisterMetaType< Z3D::ZLTSCalibrationImage::ImageState >("Z3D::ZLTSCalibrationImage::ImageState");

ZLTSCalibrationImage::ZLTSCalibrationImage(QString fileName, QObject *parent) :
    ZCalibrationImage(fileName, parent)
{

}

ZLTSCalibrationImage::~ZLTSCalibrationImage()
{

}

bool ZLTSCalibrationImage::isValid()
{
    if (ZCalibrationImage::isValid()) {
        /// if the image is valid, then try to load laser points
        QString laserFileName = QString("%1.txt").arg(m_fileName);
        FILE * fp = fopen(qPrintable(laserFileName), "r");
        if (!fp) {
            qWarning() << "Could no open laser points file" << laserFileName;
        } else {
            /// at maximum we have one laser point for each pixel in the x axis
            m_laserPoints.reserve(m_width);
            /// read file
            float x, y;
            while (2 == fscanf(fp, "%f %f", &x, &y)) { /// I need the 2!
                m_laserPoints.push_back(cv::Point2f(x, y));
            }
            fclose(fp);
        }

        /// if we don't have laser points, it's useless
        return m_laserPoints.size() > 0;
    }

    return false;
}

QImage ZLTSCalibrationImage::imageFromUrl(int viewStyle) const
{
    /// get image
    QImage image = ZCalibrationImage::imageFromUrl(viewStyle);

    /// scale we need to use to transform the laser point coordinates
    float scale = (float)image.width() / (float)m_width;

    /// Drawing into a QImage with QImage::Format_Indexed8 is not supported
    if (image.format() == QImage::Format_Indexed8)
        image = image.convertToFormat(QImage::Format_ARGB32_Premultiplied);

    /// draw all laser points
    QPainter painter(&image);
    //painter.setRenderHint(QPainter::Antialiasing);// al pedo si es un pixel nada mas
    painter.setPen(QPen(QBrush(Qt::blue), 1));
    for (unsigned int i=0; i<m_laserPoints.size(); ++i) {
        const cv::Point2f &point = m_laserPoints[i] * scale;
        painter.drawPoint(point.x, point.y);
    }

    /// draw valid laser points
    painter.setPen(QPen(QBrush(Qt::red), 1));
    for (unsigned int i=0; i<m_validLaserPoints.size(); ++i) {
        const cv::Point2f &point = m_validLaserPoints[i] * scale;
        painter.drawPoint(point.x, point.y);
    }

    return image;
}

QPixmap ZLTSCalibrationImage::pixmapFromUrl(int viewStyle) const
{
    return QPixmap::fromImage( imageFromUrl(viewStyle) );
}

bool ZLTSCalibrationImage::findPattern(ZCalibrationPatternFinder *patternFinder)
{
    bool patternFound = ZCalibrationImage::findPattern(patternFinder);

    /// empty valid laser points
    m_validLaserPoints.clear();

    if (patternFound) {
        /// calculate convex hull
        cv::convexHull(m_corners, m_convexHullPoints);

        /// update valid laser points
        m_validLaserPoints.reserve(m_laserPoints.size());///to avoid redimensioning the vector "on the fly"
        for (unsigned int i=0; i<m_laserPoints.size(); ++i) {
            /// 3rd param "false" => only checks if the point is inside a contour or not, without calculating distance
            if (0 <= cv::pointPolygonTest(m_convexHullPoints, m_laserPoints[i], false)) {
                /// negative == outside of the polygon
                m_validLaserPoints.push_back(m_laserPoints[i]);
            }
        }
    }

    return patternFound;
}

} // namespace Z3D
