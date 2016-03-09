#pragma once

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
