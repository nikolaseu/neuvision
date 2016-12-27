/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "zcameraacquisition_global.h"

#include <opencv2/core/core.hpp>

#include <QScrollArea>

class QLabel;
class QMenu;
class QSignalMapper;

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZImageViewWidget : public QScrollArea
{
    Q_OBJECT
public:
    explicit ZImageViewWidget(QWidget *parent = 0);
    ~ZImageViewWidget();
    
signals:
    void sizeChanged(QSize newSize);
    
public slots:
    void setImage(const cv::Mat &image);

private slots:
    void halfSize();
    void realSize();
    void doubleSize();
    void tripleSize();
    void zoomIn();
    void zoomOut();
    void saveImage();

    void changeColormap(int colormapId);
private:
    virtual bool eventFilter(QObject *, QEvent *);

    virtual void closeEvent(QCloseEvent *event);

    void scaleImage(double factor);
    void adjustScrollBar(QScrollBar *scrollBar, double factor);

    QLabel *m_imageLabel;
    QMenu *m_menu;

    QMenu *m_colormapsMenu;
    QSignalMapper *m_colormapSignalMapper;
    int m_colormap;

    cv::Mat m_img;
    cv::Mat m_visibleImage;

    QPixmap m_pixmap;

    double m_scaleFactor;
};

} // namespace Z3D
