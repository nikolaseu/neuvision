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

#include "zcameraacquisition_fwd.h"
#include "zcameraacquisition_global.h"

#include <opencv2/core/mat.hpp>

#include <QGraphicsView>

class QGraphicsPixmapItem;
class QMenu;
class QSignalMapper;

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZImageViewer : public QGraphicsView
{
    Q_OBJECT

public:
    ZImageViewer(QWidget *parent = nullptr);
    ~ZImageViewer() override;

public slots:
    void updateImage(Z3D::ZCameraImagePtr image);
    void updateImage(const cv::Mat &image);
    void updateImage(const cv::Mat &image, float minValue, float maxValue);
    void updateImage(const QImage &image);

    void setFitToWindow(const bool &enabled);
    void setDeleteOnClose(bool deleteOnClose);

protected slots:
    void changeColormap(int colormapId);
    void saveImage();

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void closeEvent(QCloseEvent *event) override;

protected:
    QMenu *m_contextMenu;
    QAction *m_fitToWindowAction;

    QMenu *m_colormapsMenu;
    QSignalMapper *m_colormapSignalMapper;
    int m_colormap;

    QGraphicsScene* m_scene;
    QGraphicsPixmapItem *m_pixmapItem;
    QImage m_image;

    double m_zoomFactor;
    bool m_fitToWindowEnabled;
    bool m_deleteOnClose;

    cv::Mat m_img;
    cv::Mat m_visibleImage;
};

} // namespace Z3D
