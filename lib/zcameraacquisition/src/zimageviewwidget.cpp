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

#include "zimageviewwidget.h"

#include <QAction>
#include <QDebug>
#include <QEvent>
#include <QFileDialog>
#include <QMouseEvent>
#include <QScrollBar>
#include <QSignalMapper>
#include <QMenu>
#include <QLabel>
#include <QPair>

#ifdef CV_VERSION_EPOCH
#  if CV_VERSION_EPOCH < 3
#    include "opencv2/contrib/contrib.hpp" // colormap
#  endif
#else
#  include "opencv2/imgproc.hpp" // colormap
#endif

#include "opencv2/highgui/highgui.hpp" // imwrite

namespace Z3D
{

ZImageViewWidget::ZImageViewWidget(QWidget *parent) :
    QScrollArea(parent),
    m_colormap(-1), /// colormap < 0  -->  no colormap
  m_scaleFactor(0.5)
{
    m_imageLabel = new QLabel(this);
    m_imageLabel->setBackgroundRole(QPalette::Dark);
    m_imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    m_imageLabel->setMinimumSize(QSize(300,300));
    //m_imageLabel->setScaledContents(true);

    setBackgroundRole(QPalette::Dark);
    setWidget(m_imageLabel);

    QAction *halfSizeAction = new QAction("50%", this);
    QAction *realSizeAction = new QAction("100%", this);
    QAction *doubleSizeAction = new QAction("200%", this);
    QAction *tripleSizeAction = new QAction("300%", this);
    QAction *zoomInAction = new QAction("Zoom in", this);
    QAction *zoomOutAction = new QAction("Zoom out", this);
    QAction *saveImageAction = new QAction("Save image as...", this);

    QObject::connect(halfSizeAction, SIGNAL(triggered()),
                     this, SLOT(halfSize()));
    QObject::connect(realSizeAction, SIGNAL(triggered()),
                     this, SLOT(realSize()));
    QObject::connect(doubleSizeAction, SIGNAL(triggered()),
                     this, SLOT(doubleSize()));
    QObject::connect(tripleSizeAction, SIGNAL(triggered()),
                     this, SLOT(tripleSize()));
    QObject::connect(zoomInAction, SIGNAL(triggered()),
                     this, SLOT(zoomIn()));
    QObject::connect(zoomOutAction, SIGNAL(triggered()),
                     this, SLOT(zoomOut()));
    QObject::connect(saveImageAction, SIGNAL(triggered()),
                     this, SLOT(saveImage()));

    m_menu = new QMenu(this);
    m_menu->addAction(halfSizeAction);
    m_menu->addAction(realSizeAction);
    m_menu->addAction(doubleSizeAction);
    m_menu->addAction(tripleSizeAction);
    m_menu->addSeparator();
    m_menu->addAction(zoomInAction);
    m_menu->addAction(zoomOutAction);
    m_menu->addSeparator();
    m_menu->addAction(saveImageAction);
    m_menu->addSeparator();

    /// add colormaps<int, QString>
    QList<QPair<int, QString> > colormaps;
    colormaps.push_back(qMakePair<int, QString>(-1                  , "NONE"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_AUTUMN , "AUTUMN"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_BONE   , "BONE"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_COOL   , "COOL"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_HOT    , "HOT"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_HSV    , "HSV"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_JET    , "JET"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_OCEAN  , "OCEAN"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_PINK   , "PINK"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_RAINBOW, "RAINBOW"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_SPRING , "SPRING"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_SUMMER , "SUMMER"));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_WINTER , "WINTER"));

    m_colormapsMenu = m_menu->addMenu("Colormap");
    m_colormapSignalMapper = new QSignalMapper(this);
    for (int i = 0; i<colormaps.size(); ++i) {
        QAction *colormapAction = new QAction(colormaps[i].second, this);
        connect(colormapAction, SIGNAL(triggered()), m_colormapSignalMapper, SLOT(map()));
        m_colormapSignalMapper->setMapping(colormapAction, colormaps[i].first);
        m_colormapsMenu->addAction(colormapAction);
    }

    QObject::connect(m_colormapSignalMapper, SIGNAL(mapped(int)),
                     this, SLOT(changeColormap(int)));
}

ZImageViewWidget::~ZImageViewWidget()
{
    qDebug() << Q_FUNC_INFO;
}

void ZImageViewWidget::setImage(const cv::Mat &image)
{
    image.copyTo(m_img);

    if (m_img.channels() == 1) {
        /// colormap only enabled for grayscale images
        m_colormapsMenu->setEnabled(true);
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
        /// colormap only enabled for grayscale images
        m_colormapsMenu->setEnabled(false);
        m_img.convertTo(m_visibleImage, CV_8UC3);
    } else {
        qCritical() << "unsupported image";
        return;
    }

    /// last check, later we assume we have rgb 8 bit per channel!
    if (m_visibleImage.type() != CV_8UC3) {
        qDebug() << "converting image to CV_8UC3 from type:" << m_visibleImage.type();
        m_visibleImage.convertTo(m_visibleImage, CV_8UC3);
    }

    m_pixmap = QPixmap::fromImage(
                QImage(m_visibleImage.data,
                       m_visibleImage.size().width,
                       m_visibleImage.size().height,
                       QImage::Format_RGB888)
                //.rgbSwapped() /// because we need to swap BGR to RGB
                );

    QSize newSize = m_scaleFactor * m_pixmap.size();

    m_imageLabel->setPixmap( m_pixmap.scaled(newSize,
                                             Qt::KeepAspectRatio,
                                             Qt::FastTransformation) );

    m_imageLabel->resize(newSize);
}

void ZImageViewWidget::changeColormap(int colormapId)
{
    m_colormap = colormapId;
    setImage(m_img);
}

void ZImageViewWidget::halfSize()
{
    scaleImage(0.5 / m_scaleFactor);
}

void ZImageViewWidget::realSize()
{
    scaleImage(1.0 / m_scaleFactor);
}

void ZImageViewWidget::doubleSize()
{
    scaleImage(2.0 / m_scaleFactor);
}

void ZImageViewWidget::tripleSize()
{
    scaleImage(3.0 / m_scaleFactor);
}

void ZImageViewWidget::zoomIn()
{
    scaleImage(1.2);
}

void ZImageViewWidget::zoomOut()
{
    scaleImage(0.8);
}

void ZImageViewWidget::saveImage()
{
    /// get file name
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save image as..."),
                                                    ".",
                                                    tr("PNG image (*.png);; BMP image (*.bmp)"));

    if (fileName.isEmpty() || fileName.isNull())
        return;

    /// save image as currently seen (w/colormap for example), not the original
    try {
        cv::imwrite(qPrintable(fileName), m_visibleImage);
    } catch (...) {
        qCritical() << "unable to save image to file" << fileName;
    }
}

bool ZImageViewWidget::eventFilter(QObject *obj, QEvent *event)
{
    if(event->type() == QEvent::ContextMenu) {
        if (m_imageLabel->pixmap()) {
            /// only show if some image is set
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*> (event);
            m_menu->exec(mouseEvent->globalPos());
        }
        return true;
    }
    else
        return QScrollArea::eventFilter(obj, event);
}

void ZImageViewWidget::closeEvent(QCloseEvent * /*event*/)
{
    /// delete on close
    deleteLater();
}

void ZImageViewWidget::scaleImage(double factor)
{
    Q_ASSERT(m_imageLabel->pixmap());
    m_scaleFactor *= factor;

    m_imageLabel->setPixmap( m_pixmap.scaled(m_scaleFactor * m_pixmap.size(),
                                             Qt::KeepAspectRatio,
                                             Qt::FastTransformation) );

    m_imageLabel->resize(m_imageLabel->pixmap()->size());

    adjustScrollBar(this->horizontalScrollBar(), factor);
    adjustScrollBar(this->verticalScrollBar(), factor);

    //! TODO: is the margin required by the layout? try to remove it
    // 1px margin every side.
    //resize(m_imageLabel->size() + QSize(2,2));

    emit sizeChanged(size());
}

void ZImageViewWidget::adjustScrollBar(QScrollBar *scrollBar, double factor)
{
    scrollBar->setValue(int(factor * scrollBar->value()
                            + ((factor - 1) * scrollBar->pageStep()/2)));
}

} // namespace Z3D
