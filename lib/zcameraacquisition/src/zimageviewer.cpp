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

#include "ZCameraAcquisition/zimageviewer.h"

#include "ZCameraAcquisition/zcameraimage.h"

#include "ZCore/zlogging.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QGraphicsPixmapItem>
#include <QMenu>
#include <QMouseEvent>
#include <QScrollBar>
#include <qmath.h>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcameraacquisition", QtInfoMsg)

namespace Z3D
{

ZImageViewer::ZImageViewer(QWidget *parent)
    : QGraphicsView(parent)
    , m_colormap(-1) /// colormap < 0  -->  no colormap
    , m_scene(new QGraphicsScene(this))
    , m_pixmapItem(new QGraphicsPixmapItem())
    , m_zoomFactor(0)
    , m_fitToWindowEnabled(true)
    , m_deleteOnClose(false)
{
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setDragMode(QGraphicsView::ScrollHandDrag);

    //setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    setMouseTracking(true);

    setScene(m_scene);

    /// create context menu
    m_contextMenu = new QMenu(this);

    /// fit to window action
    m_fitToWindowAction = new QAction(tr("Fit to window"), m_contextMenu);
    m_fitToWindowAction->setCheckable(true);
    m_fitToWindowAction->setChecked(m_fitToWindowEnabled);
    QObject::connect(m_fitToWindowAction, &QAction::toggled,
                     this, &ZImageViewer::setFitToWindow);
    m_contextMenu->addAction(m_fitToWindowAction);

    /// add colormaps<int, QString>
    const std::vector<std::pair<int, QString> > colormaps {
      std::make_pair<int, QString>(-1                  , tr("None")),
      std::make_pair<int, QString>(cv::COLORMAP_AUTUMN , tr("Autumn")),
      std::make_pair<int, QString>(cv::COLORMAP_BONE   , tr("Bone")),
      std::make_pair<int, QString>(cv::COLORMAP_COOL   , tr("Cool")),
      std::make_pair<int, QString>(cv::COLORMAP_HOT    , tr("Hot")),
      std::make_pair<int, QString>(cv::COLORMAP_HSV    , tr("HSV")),
      std::make_pair<int, QString>(cv::COLORMAP_JET    , tr("Jet")),
      std::make_pair<int, QString>(cv::COLORMAP_OCEAN  , tr("Ocean")),
      std::make_pair<int, QString>(cv::COLORMAP_PINK   , tr("Pink")),
      std::make_pair<int, QString>(cv::COLORMAP_RAINBOW, tr("Rainbow")),
      std::make_pair<int, QString>(cv::COLORMAP_SPRING , tr("Spring")),
      std::make_pair<int, QString>(cv::COLORMAP_SUMMER , tr("Summer")),
      std::make_pair<int, QString>(cv::COLORMAP_WINTER , tr("Winter")),
    };

    /// colormaps menu
    m_contextMenu->addSeparator();
    m_colormapsMenu = m_contextMenu->addMenu(tr("Colormap"));
    m_colormapsMenu->setEnabled(false);
    for (auto &colormap : colormaps) {
        auto colormapAction = new QAction(colormap.second, this);
        colormapAction->setCheckable(true);
        colormapAction->setChecked(colormap.first == m_colormap);
        QObject::connect(colormapAction, &QAction::toggled,
                         this, std::bind(&ZImageViewer::changeColormap, this, std::placeholders::_1, colormap.first));
        m_colormapsMenu->addAction(colormapAction);
        m_colormapActions.insert(colormap.first, colormapAction);
    }

    /// save image action
    auto saveImageAction = new QAction(tr("Save image as..."), m_contextMenu);
    QObject::connect(saveImageAction, &QAction::triggered,
                     this, &ZImageViewer::saveImage);
    m_contextMenu->addSeparator();
    m_contextMenu->addAction(saveImageAction);

    /// add pixmap item
    QPixmap image(":/zcameraacquisition/empty.png");
    m_pixmapItem->setPixmap(image);
    m_scene->addItem(m_pixmapItem);
}

ZImageViewer::~ZImageViewer ()
{
    zDebug() << Q_FUNC_INFO;
}

void ZImageViewer::setFitToWindow(const bool &enabled)
{
    m_fitToWindowEnabled = enabled;

    if (m_fitToWindowEnabled != m_fitToWindowAction->isChecked())
        m_fitToWindowAction->setChecked(m_fitToWindowEnabled);

    if(m_fitToWindowEnabled) {
        fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
    } else {
        /// update current zoom factor
        /// we need to calculate the inverse of what we do when doing zoom in/out
        /// qreal scale = qPow(qreal(2), (qreal)m_zoomFactor / qreal(10));
        qreal scale = transform().m11();
        /// get exponent -> log_2(scale) == log(x)/log(2)
        qreal exp = log(scale) /  log(qreal(2));
        m_zoomFactor = exp * qreal(10); /// este 10 tiene que ser el mismo que uso en wheelEvent!
    }
}

void ZImageViewer::setDeleteOnClose(bool deleteOnClose)
{
    m_deleteOnClose = deleteOnClose;
}

void ZImageViewer::changeColormap(bool enabled, int colormapId)
{
    if (!enabled) {
        // reset
        changeColormap(true, -1);
        return;
    }

    if (m_colormap == colormapId) {
        return;
    }

    /// uncheck previous
    QAction *colormapActionPrevious = m_colormapActions[m_colormap];
    const bool signalsWereBlockedPrevious = colormapActionPrevious->blockSignals(true);
    colormapActionPrevious->setChecked(false);
    colormapActionPrevious->blockSignals(signalsWereBlockedPrevious);

    /// update current
    m_colormap = colormapId;

    /// check current
    QAction *colormapActionNew = m_colormapActions[m_colormap];
    const bool signalsWereBlockedNew = colormapActionNew->blockSignals(true);
    colormapActionNew->setChecked(true);
    colormapActionNew->blockSignals(signalsWereBlockedNew);

    /// if we were displaying an image, update to view current colormap
    if (m_img.cols) {
        updateImage(m_img);
    }
}

void ZImageViewer::saveImage()
{
    /// get file name
    auto fileName = QFileDialog::getSaveFileName(this,
                                                 tr("Save image as..."),
                                                 "image.png",
                                                 tr("Images (*.png *.jpg *.jpeg *.bmp)"));

    if (fileName.isEmpty() || fileName.isNull())
        return;

    /// save image as currently seen (w/colormap for example), not the original
    /// format=0 (use filename extension)
    /// quality=100 (maximum)
    if (!m_image.save(fileName /*, 0, 100*/)) {
        zCritical() << "unable to save image to file" << fileName;
        //QMessageBox::
    }
}

void ZImageViewer::mousePressEvent(QMouseEvent* event)
{ 
    if(event->button() == Qt::RightButton) {
        m_contextMenu->exec(event->globalPos());
    } else {
        QGraphicsView::mousePressEvent(event);
    }
}

void ZImageViewer::wheelEvent(QWheelEvent* event)
{   
    if (event->modifiers() & Qt::ControlModifier) {
        /// disable fit to window
        setFitToWindow(false);

        const QPoint numPixels = event->pixelDelta();
        const QPoint numDegrees = event->angleDelta() / 8;

        int delta = 0;
        if (!numPixels.isNull()) {
            delta = numPixels.y();
        }
        else if (!numDegrees.isNull()) {
            QPoint numSteps = numDegrees / 15;
            delta = numSteps.y();
        }

        if (delta > 0) {
            m_zoomFactor++;
        }
        else {
            m_zoomFactor--;
        }

        const qreal scale = qPow(qreal(2), qreal(m_zoomFactor) / qreal(10)); /// este 10 tiene que ser el mismo que uso en setFitToWindow!

        QTransform matrix;
        matrix.scale(scale, scale);
        setTransform(matrix);

        event->accept();
    } else {
        QGraphicsView::wheelEvent(event);
    }
}

void ZImageViewer::resizeEvent(QResizeEvent* event)
{
    if(m_fitToWindowEnabled) {
        fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
    }

    /// call the superclass resize so the scrollbars are updated correctly
    QGraphicsView::resizeEvent(event);
}

void ZImageViewer::closeEvent(QCloseEvent *event)
{
    /// if delete on close is enabled, schedule delete
    if (m_deleteOnClose)
        deleteLater();

    QGraphicsView::closeEvent(event);
}

void ZImageViewer::updateImage(const ZCameraImagePtr &image)
{
    if (image)
        updateImage(image->cvMat());
    else
        updateImage(cv::Mat());
}

void ZImageViewer::updateImage(const cv::Mat &image)
{
    if (!image.cols) {
        /// empty view
        updateImage(QImage());

        ///
        return;
    }

    m_img = image;

    /// colormap only enabled for grayscale images
    m_colormapsMenu->setEnabled(m_img.channels() == 1);

    if (m_img.channels() == 1) {
        /// something goes wrong when using 16 bit images
        if (m_img.type() != CV_8UC1)
            m_img.convertTo(m_img, CV_8UC1);

        if (m_colormap >= 0) {
            /// apply the colormap
            cv::applyColorMap(m_img, m_visibleImage, m_colormap);
        } else {
            /// convert to rgb
            cv::cvtColor(m_img, m_visibleImage, cv::COLOR_GRAY2BGR); //CV_GRAY2RGB);
        }
    } else if(m_img.channels() == 3) {
        m_img.convertTo(m_visibleImage, CV_8UC3);
    } else if(m_img.channels() == 4) {
        QImage qImage(m_img.data,
                      m_img.cols,
                      m_img.rows,
                      m_img.step,
                      QImage::Format_RGBX8888);

        updateImage(qImage);

        return;
    } else {
        zCritical() << "unsupported image type:" << m_img.type() << "size:" << m_img.cols << "x" << m_img.rows;
        return;
    }

    /// last check, later we assume we have rgb 8 bit per channel!
    if (m_visibleImage.type() != CV_8UC3) {
        zDebug() << "converting image to CV_8UC3 from type:" << m_visibleImage.type();
        m_visibleImage.convertTo(m_visibleImage, CV_8UC3);
    }

    QImage qImage(m_visibleImage.data,
                  m_visibleImage.cols,
                  m_visibleImage.rows,
                  m_visibleImage.step,
                  QImage::Format_RGB888);

    /// we need to swap BGR to RGB
    //qImage = qImage.rgbSwapped();

    updateImage(qImage);
}

void ZImageViewer::updateImage(const cv::Mat &image, float minValue, float maxValue)
{
    if (m_img.channels() != 1) {
        zCritical() << "cannot show image with min/max range because it has more than one channel!";
        return;
    }

    /// convert to "visible" image to show in window
    const float range = maxValue - minValue;
    cv::Mat img;
    //! FIXME this loses a lot of detail
    image.convertTo(img, CV_8UC1, 256/range, -minValue * 256/range);

    updateImage(img);
}

void ZImageViewer::updateImage(const QImage &image)
{
    const bool haveToForceFitImageToWindow = m_fitToWindowEnabled && m_image.size() != image.size();

    m_image = image;

    auto pixmap = QPixmap::fromImage(m_image);

    m_pixmapItem->setPixmap(pixmap);

    if (haveToForceFitImageToWindow) {
        fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
    }
}

} // namespace Z3D
