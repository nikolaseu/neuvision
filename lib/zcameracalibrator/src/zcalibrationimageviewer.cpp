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

#include "zcalibrationimageviewer.h"

#include <QDebug>
#include <QGraphicsEllipseItem>
#include <QGraphicsItem>
#include <QGraphicsPolygonItem>

namespace Z3D
{

ZCalibrationImageViewer::ZCalibrationImageViewer(QWidget *parent)
    : ZImageViewer(parent)
    , m_displayMode(ShowMarkers)
    , m_markerPolygon(8)
{
    m_markerPolygon.append(QPointF(-4, -4));
    m_markerPolygon.append(QPointF( 0,  0));
    m_markerPolygon.append(QPointF(-4,  4));
    m_markerPolygon.append(QPointF( 0,  0));
    m_markerPolygon.append(QPointF( 4, -4));
    m_markerPolygon.append(QPointF( 0,  0));
    m_markerPolygon.append(QPointF( 4,  4));
    m_markerPolygon.append(QPointF( 0,  0));
}

ZCalibrationImageViewer::~ZCalibrationImageViewer()
{

}

void ZCalibrationImageViewer::setDisplayMode(ZCalibrationImageViewer::DisplayMode displayMode)
{
    if (m_displayMode == displayMode)
        return;

    m_displayMode = displayMode;

    for (std::vector<QGraphicsItem*>::iterator it = m_calibrationPoints.begin(); it != m_calibrationPoints.end(); ++it) {
        QGraphicsItem *item = *it;
        item->setVisible(m_displayMode != NoMarkers);
    }

    for (std::vector<QGraphicsItem*>::iterator it = m_calibrationCoords.begin(); it != m_calibrationCoords.end(); ++it) {
        QGraphicsItem *item = *it;
        item->setVisible(m_displayMode == ShowMarkersAndCoords);
    }
}

void ZCalibrationImageViewer::updateCalibrationImage(ZCalibrationImagePtr image)
{
    for (std::vector<QGraphicsItem*>::iterator it = m_calibrationPoints.begin(); it != m_calibrationPoints.end(); ++it) {
        m_scene->removeItem(*it);
        delete *it;
    }
    m_calibrationPoints.clear();

    for (std::vector<QGraphicsItem*>::iterator it = m_calibrationCoords.begin(); it != m_calibrationCoords.end(); ++it) {
        m_scene->removeItem(*it);
        delete *it;
    }
    m_calibrationCoords.clear();

    /// use cv::Mat as image, it allows to apply colormap
    ZImageViewer::updateImage(image->mat());

    const std::vector<cv::Point2f> &points = image->detectedPoints();
    if (points.empty())
        return;

    const std::vector<cv::Point3f> &patternPoints = image->detectedPointsInPatternCoordinates();

    m_calibrationPoints.reserve( 2*points.size());
    m_calibrationCoords.reserve( points.size());

    int outerCircleRadius = 4; /// actually half radius
    int innerCircleRadius = 2; /// actually half radius

    QFont font;
    font.setPixelSize(11);
    font.setWeight(QFont::Thin);

    auto itW = patternPoints.cbegin();
    for (auto it = points.cbegin(); it != points.cend(); ++it, ++itW) {
        if (false) {
            QGraphicsPolygonItem *item = new QGraphicsPolygonItem(m_markerPolygon);
            item->setPen(QPen(QBrush(Qt::blue), 3));
            item->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
            item->setPos(it->x, it->y);
            item->setVisible(m_displayMode != NoMarkers);
            m_scene->addItem(item);
            m_calibrationPoints.push_back(item);

            QGraphicsPolygonItem *item2 = new QGraphicsPolygonItem(m_markerPolygon);
            item2->setPen(QPen(QBrush(Qt::green), 1));
            item2->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
            item2->setPos(it->x, it->y);
            item2->setVisible(m_displayMode != NoMarkers);
            m_scene->addItem(item2);
            m_calibrationPoints.push_back(item2);
        } else {
            QGraphicsEllipseItem *item = new QGraphicsEllipseItem(-outerCircleRadius, -outerCircleRadius, 2*outerCircleRadius, 2*outerCircleRadius);
            item->setPen(QPen(QBrush(Qt::blue), outerCircleRadius+2));
            item->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
            item->setPos(it->x, it->y);
            item->setVisible(m_displayMode != NoMarkers);
            m_scene->addItem(item);
            m_calibrationPoints.push_back(item);

            QGraphicsEllipseItem *item2 = new QGraphicsEllipseItem(-innerCircleRadius, -innerCircleRadius, 2*innerCircleRadius, 2*innerCircleRadius);
            if ((itW->x == 0.f && itW->y == 0.f) ||
                (itW->x == 2.f && itW->y == 0.f) ||
                (itW->x == 0.f && itW->y == 1.f))
                item2->setPen(QPen(QBrush(Qt::red), innerCircleRadius+2));
            else
                item2->setPen(QPen(QBrush(Qt::green), innerCircleRadius+2));
            item2->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
            item2->setPos(it->x, it->y);
            item2->setVisible(m_displayMode != NoMarkers);
            m_scene->addItem(item2);
            m_calibrationPoints.push_back(item2);

            QString text = QString("%1,%2").arg(itW->x).arg(itW->y);
            QGraphicsSimpleTextItem *item3 = new QGraphicsSimpleTextItem(text);
            item3->setFont(font);
            item3->setBrush(QBrush(Qt::red));
            item3->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
            item3->setPos(it->x, it->y);
            item3->setVisible(m_displayMode == ShowMarkersAndCoords);
            m_scene->addItem(item3);
            m_calibrationCoords.push_back(item3);
        }
    }
}

} // namespace Z3D
