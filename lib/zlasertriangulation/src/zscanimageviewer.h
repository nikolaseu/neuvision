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

﻿#pragma once

#include "zlasertriangulation_global.h"
#include "zscanimageviewerfilter.h"

#include <Z3DCameraAcquisition>

#include <QGraphicsPixmapItem>
#include <QGraphicsView>

namespace Z3D
{

class Z3D_LASERTRIANGULATION_SHARED_EXPORT ScanImageViewer : public QGraphicsView
{
    Q_OBJECT

    Q_PROPERTY(bool scaleValues READ scaleValues WRITE setScaleValues NOTIFY scaleValuesChanged)
    Q_PROPERTY(double minimum READ minimum WRITE setMinimum NOTIFY minimumChanged)
    Q_PROPERTY(double maximum READ maximum WRITE setMaximum NOTIFY maximumChanged)
    Q_PROPERTY(int visibleImageCount WRITE setVisibleImageCount NOTIFY visibleImageCountChanged)

public:
    enum DisplayMode {
        NormalDisplay = 0,
        LineSubtractionDisplay,
        LineSubtractionRansacDisplay,
        UnsharpMaskingDisplay
    };

    ScanImageViewer(QWidget *parent = NULL);
    ~ScanImageViewer();

    bool scaleValues() const;
    double minimum() const;
    double maximum() const;

public slots:
    void updateImage(Z3D::ZImageGrayscale::Ptr scanImage);

    void setFitToWindow(const bool &enabled);

    void setVisibleImageCount(int count);

    void setScaleValues(bool arg);
    void setMinimum(double arg);
    void setMaximum(double arg);
    void setAutoScale();

    void setDisplayMode(DisplayMode displayMode);

signals:
    void scaleValuesChanged(bool arg);
    void minimumChanged(double arg);
    void maximumChanged(double arg);

    void visibleImageCountChanged(int arg);

protected slots:
    void changeColormap(int colormapId);

protected:
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void wheelEvent(QWheelEvent* event);
    virtual void resizeEvent(QResizeEvent* event);

private:
    QMenu *m_contextMenu;
    QAction *m_fitToWindowAction;

    QMenu *m_colormapsMenu;
    QSignalMapper *m_colormapSignalMapper;
    int m_colormap;

    QGraphicsScene* m_scene;
    QGraphicsView* m_view;
    QList<QGraphicsPixmapItem *> m_pixmapItems;
    int m_visiblePixmapCount;

    double m_zoomFactor;
    bool   m_fitToWindowEnabled;

    bool m_scaleValues;
    bool m_autoScaleRequested;
    double m_minimum;
    double m_maximum;

    DisplayMode m_currentDisplayMode;
    ScanImageViewerFilter *m_currentDisplayFilter;

    cv::Mat m_currentJoinedImage;
    int m_currentJoinedImageRowIndex;
};

} // namespace Z3D
