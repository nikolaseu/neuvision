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
}

void ZCalibrationImageViewer::updateCalibrationImage(ZCalibrationImage::Ptr image)
{
    for (std::vector<QGraphicsItem*>::iterator it = m_calibrationPoints.begin(); it != m_calibrationPoints.end(); ++it) {
        m_scene->removeItem(*it);
        delete *it;
    }
    m_calibrationPoints.clear();

    /// use cv::Mat as image, it allows to apply colormap
    ZImageViewer::updateImage(image->mat());

    const std::vector<cv::Point2f> &points = image->detectedPoints();
    if (points.empty())
        return;

    m_calibrationPoints.reserve( 2*points.size());

    int outerCircleRadius = 4; /// actually half radius
    int innerCircleRadius = 2; /// actually half radius

    for (std::vector<cv::Point2f>::const_iterator it = points.begin(); it != points.end(); ++it) {
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
            if (it == points.begin())
                item2->setPen(QPen(QBrush(Qt::red), innerCircleRadius+2));
            else
                item2->setPen(QPen(QBrush(Qt::green), innerCircleRadius+2));
            item2->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
            item2->setPos(it->x, it->y);
            item2->setVisible(m_displayMode != NoMarkers);
            m_scene->addItem(item2);
            m_calibrationPoints.push_back(item2);
        }
    }
}

} // namespace Z3D
