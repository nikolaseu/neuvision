#include "zscanimageviewer.h"

#include <QDebug>
#include <QFileDialog>
#include <QGLWidget>
#include <QMenu>
#include <QMouseEvent>
#include <QScrollBar>
#include <QSignalMapper>

#include <qmath.h>

namespace Z3D
{


ScanImageViewer::ScanImageViewer(QWidget *parent)
    : QGraphicsView(parent)
    , m_colormap(cv::COLORMAP_JET) /// start with jet colormap
    , m_zoomFactor(0)
    , m_fitToWindowEnabled(false)
    , m_scene(new QGraphicsScene())
    , m_visiblePixmapCount(-1)
    , m_scaleValues(false)
    , m_autoScaleRequested(true)
    , m_minimum(-1.)
    , m_maximum(-1.)
    , m_currentDisplayMode(NormalDisplay)
    , m_currentDisplayFilter(new ScanImageViewerFilter())
    , m_currentJoinedImageRowIndex(0)
{
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setDragMode(QGraphicsView::ScrollHandDrag);

    /// to render using OpenGL
    //setViewport(new QGLWidget());

    //setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    setMouseTracking(true);

    setScene(m_scene);

    /// create context menu
    m_contextMenu = new QMenu(this);

    /// fit to window action
    m_fitToWindowAction = new QAction(tr("Fit to window"), m_contextMenu);
    m_fitToWindowAction->setCheckable(true);
    m_fitToWindowAction->setChecked(m_fitToWindowEnabled);
    QObject::connect(m_fitToWindowAction, SIGNAL(toggled(bool)),
                     this, SLOT(setFitToWindow(bool)));
    m_contextMenu->addAction(m_fitToWindowAction);

    /// add colormaps<int, QString>
    QList<QPair<int, QString> > colormaps;
    colormaps.push_back(qMakePair<int, QString>(-1                  , tr("None")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_AUTUMN , tr("Autumn")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_BONE   , tr("Bone")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_COOL   , tr("Cool")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_HOT    , tr("Hot")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_HSV    , tr("HSV")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_JET    , tr("Jet")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_OCEAN  , tr("Ocean")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_PINK   , tr("Pink")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_RAINBOW, tr("Rainbow")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_SPRING , tr("Spring")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_SUMMER , tr("Summer")));
    colormaps.push_back(qMakePair<int, QString>(cv::COLORMAP_WINTER , tr("Winter")));

    /// colormaps menu
    m_contextMenu->addSeparator();
    m_colormapsMenu = m_contextMenu->addMenu(tr("Colormap"));
    m_colormapsMenu->setEnabled(false);
    m_colormapSignalMapper = new QSignalMapper(this);
    for (int i = 0; i<colormaps.size(); ++i) {
        QAction *colormapAction = new QAction(colormaps[i].second, this);
        colormapAction->setCheckable(true);
        if (colormaps[i].first == m_colormap) {
            colormapAction->setChecked(true);
        }
        QObject::connect(colormapAction, SIGNAL(triggered()), m_colormapSignalMapper, SLOT(map()));
        m_colormapSignalMapper->setMapping(colormapAction, colormaps[i].first);
        m_colormapsMenu->addAction(colormapAction);
    }

    QObject::connect(m_colormapSignalMapper, SIGNAL(mapped(int)),
                     this, SLOT(changeColormap(int)));
/*
    /// save image action
    QAction *saveImageAction = new QAction(tr("Save image as..."), m_contextMenu);
    QObject::connect(saveImageAction, SIGNAL(triggered()),
                     this, SLOT(saveImage()));
    m_contextMenu->addSeparator();
    m_contextMenu->addAction(saveImageAction);
*/
    /// add pixmap items
    QPixmap image(":/empty.png");
    m_pixmapItems.reserve(m_visiblePixmapCount);
    for (int i=0; i<m_visiblePixmapCount; ++i) {
        QGraphicsPixmapItem *m_pixmapItem = new QGraphicsPixmapItem();
        m_pixmapItem->setPixmap(image);
        m_pixmapItems << m_pixmapItem;
        m_scene->addItem(m_pixmapItem);
    }
}

ScanImageViewer::~ScanImageViewer ()
{

}

bool ScanImageViewer::scaleValues() const
{
    return m_scaleValues;
}

void ScanImageViewer::setFitToWindow(const bool &enabled)
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
        qreal scale = matrix().m11();
        /// get exponent -> log_2(scale) == log(x)/log(2)
        qreal exp = log(scale) /  log(qreal(2));
        m_zoomFactor = exp * qreal(10); /// este 10 tiene que ser el mismo que uso en wheelEvent!
    }
}

void ScanImageViewer::setVisibleImageCount(int count)
{
    if (m_visiblePixmapCount != count) {
        if (m_visiblePixmapCount < count) {
            /// add pixmaps
            for (int i=m_visiblePixmapCount; i<count; ++i) {
                QGraphicsPixmapItem *m_pixmapItem = new QGraphicsPixmapItem();
                //m_pixmapItem->setPixmap(image);
                m_pixmapItems << m_pixmapItem;
                m_scene->addItem(m_pixmapItem);
            }
        } else {
            /// remove pixmaps
            for (int i=m_visiblePixmapCount-1; i>count; --i) {
                QGraphicsPixmapItem *m_pixmapItem = m_pixmapItems[i];
                m_scene->removeItem(m_pixmapItem);
                m_pixmapItems.removeAt(i);
                delete m_pixmapItem;
            }
        }

        m_visiblePixmapCount = count;
        emit visibleImageCountChanged(count);
    }
}

void ScanImageViewer::setScaleValues(bool arg)
{
    if (m_scaleValues == arg)
        return;

    m_scaleValues = arg;
    emit scaleValuesChanged(arg);
}

void ScanImageViewer::setMinimum(double arg)
{
    if (m_minimum != arg) {
        m_minimum = arg;
        emit minimumChanged(arg);
    }
}

void ScanImageViewer::setMaximum(double arg)
{
    if (m_maximum != arg) {
        m_maximum = arg;
        emit maximumChanged(arg);
    }
}

void ScanImageViewer::setAutoScale()
{
    m_autoScaleRequested = true;
}

void ScanImageViewer::setDisplayMode(ScanImageViewer::DisplayMode displayMode)
{
    if (m_currentDisplayMode != displayMode) {
        m_currentDisplayMode = displayMode;

        /// delete after (i.e. next event loop)
        m_currentDisplayFilter->deleteLater();

        switch (m_currentDisplayMode) {
        case LineSubtractionDisplay:
            m_currentDisplayFilter = new LineSubtractionScanImageFilter();
            break;
        case LineSubtractionRansacDisplay:
            m_currentDisplayFilter = new LineSubtractionRansacScanImageViewerFilter();
            break;
        case UnsharpMaskingDisplay:
            m_currentDisplayFilter = new UnsharpMaskingScanImageViewerFilter();
            break;
        case NormalDisplay:
        default:
            m_currentDisplayFilter = new ScanImageViewerFilter();
        }

        //emit currentDisplayModeChanged(displayMode);
    }
}

void ScanImageViewer::changeColormap(int colormapId)
{
    if (m_colormap == colormapId)
        return;

    /// uncheck previous
    QAction *colormapAction = (QAction*) m_colormapSignalMapper->mapping(m_colormap);
    colormapAction->setChecked(false);

    /// update current
    m_colormap = colormapId;

    /// check current
    colormapAction = (QAction*) m_colormapSignalMapper->mapping(m_colormap);
    colormapAction->setChecked(true);

    /// if we were displaying an image, update to view current colormap
//    if (m_img.cols)
//        updateImage(m_img);
}


void ScanImageViewer::mousePressEvent(QMouseEvent* event)
{ 
    if(event->button() == Qt::RightButton) {
        m_contextMenu->exec(event->globalPos());
    } else {
        QGraphicsView::mousePressEvent(event);
    }
}

void ScanImageViewer::wheelEvent(QWheelEvent* event)
{   
    if (event->modifiers() & Qt::ControlModifier) {
        /// disable fit to window
        setFitToWindow(false);

        //0 < event->delta() ? zoomIn() : zoomOut();
        if (event->delta() > 0)
            m_zoomFactor++;
        else
            m_zoomFactor--;

        qreal scale = qPow(qreal(2), qreal(m_zoomFactor) / qreal(10));

        QMatrix matrix;
        matrix.scale(scale, scale);
        setMatrix(matrix);

        event->accept();
    } else {
        QGraphicsView::wheelEvent(event);
    }
}

void ScanImageViewer::resizeEvent(QResizeEvent* event)
{
    if(m_fitToWindowEnabled) {
        fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
    }

    /// Call the superclass resize so the scrollbars are updated correctly
    QGraphicsView::resizeEvent(event);
}

double ScanImageViewer::minimum() const
{
    return m_minimum;
}

double ScanImageViewer::maximum() const
{
    return m_maximum;
}

void ScanImageViewer::updateImage(ZImageGrayscale::Ptr scanImage)
{
    ZImageGrayscale::Ptr image = m_currentDisplayFilter->processImage(scanImage);

    if (!image)
        return;

    cv::Mat m_img = image->cvMat();

    const int m_imgRows = m_img.rows;
    const int m_imgCols = m_img.cols;

    const int m_minImageRowsToProcess = 10;

    const int imagesToJoinAndProcess = qMax(1, m_minImageRowsToProcess / m_imgRows);
    m_currentJoinedImageRowIndex = m_currentJoinedImageRowIndex % imagesToJoinAndProcess;
    if (m_currentJoinedImage.rows != m_imgRows * imagesToJoinAndProcess) {
        qDebug() << "creating new buffer image...";
        m_currentJoinedImageRowIndex = 0;
        m_currentJoinedImage.create(m_imgRows * imagesToJoinAndProcess, m_imgCols, CV_8UC3);
    }

    int startRow = m_currentJoinedImageRowIndex * m_imgRows;
    int endRow = startRow + m_imgRows; /// exclusive endRow (i.e. does not include this "end" row)
    cv::Mat m_visibleImage = m_currentJoinedImage.rowRange(startRow, endRow);

    if (m_visiblePixmapCount < 0) {
        setVisibleImageCount(qMax(1, 2000 / (m_imgRows * imagesToJoinAndProcess)));
    }

    /// colormap only enabled for grayscale images (this is a scan but whatever...)
    m_colormapsMenu->setEnabled(m_img.channels() == 1);
/*
    if (m_img.type() == CV_16UC1 || m_img.type() == CV_8UC1) {
        cv::medianBlur(m_img, m_img, 3);
    } else {
        qWarning() << "only CV_16UC1 or CV_8UC1 images implemented for the moment...";
    }
*/
    if (m_img.channels() == 1) {
        if (m_scaleValues) {

            if (m_autoScaleRequested || m_minimum < 0 || m_maximum < 0) {
                /// we need to update the range (min and max)
                cv::Mat mask = m_img > 0;

                cv::minMaxIdx(m_img, &m_minimum, &m_maximum);
                if (m_maximum != 0) {

                    /// create image with max values where the value was zero
                    cv::Mat adjMap = cv::Mat(m_img.size(), m_img.type(), cv::Scalar(m_maximum));
                    m_img.copyTo(adjMap, mask);

                    /// now we can get the real min (without zero!)
                    cv::minMaxIdx(adjMap, &m_minimum, &m_maximum);

                    emit minimumChanged(m_minimum);
                    emit maximumChanged(m_maximum);
                }

                m_autoScaleRequested = false;
            }

            /// scale colormap using min and max values
            cv::Mat mask = m_img >= m_minimum;

            cv::Mat adjMap = cv::Mat(m_img.size(), m_img.type(), cv::Scalar(m_minimum));
            m_img.copyTo(adjMap, mask);

            /// subtract minimum
            adjMap -= m_minimum;

            /// scale color range
            cv::convertScaleAbs(adjMap, m_img, 255 / (m_maximum - m_minimum));
        }

        /// something goes wrong when using 16 bit images
        if (m_img.type() != CV_8UC1)
            m_img.convertTo(m_img, CV_8UC1);

        if (m_colormap >= 0) {
            /// apply the colormap
            cv::applyColorMap(m_img, m_visibleImage, m_colormap);
        } else {
            /// convert to rgb
            cv::cvtColor(m_img, m_visibleImage, cv::COLOR_GRAY2BGR);
        }
    } else if(m_img.channels() == 3) {
        m_img.convertTo(m_visibleImage, CV_8UC3);
    } else {
        qCritical() << "unsupported image";
        return;
    }

    /// last check, later we assume we have rgb 8 bit per channel!
    if (m_visibleImage.type() != CV_8UC3) {
        qWarning() << "this should not happen! converting image to CV_8UC3 from type:" << m_visibleImage.type();
        m_visibleImage.convertTo(m_visibleImage, CV_8UC3);
    }

    if (m_currentJoinedImageRowIndex == imagesToJoinAndProcess - 1) {
        QImage qImage(m_currentJoinedImage.data, m_currentJoinedImage.cols, m_currentJoinedImage.rows, QImage::Format_RGB888);
        //QImage qImage(m_visibleImage.data, m_visibleImage.cols, m_visibleImage.rows, QImage::Format_RGB888);

        //qImage = qImage.rgbSwapped(); /// because we need to swap BGR to RGB

        QPixmap pixmap = QPixmap::fromImage(qImage);

        int pixmapIndex = image->number() % m_visiblePixmapCount;

        QGraphicsPixmapItem *m_pixmapItem = m_pixmapItems[pixmapIndex];
        m_pixmapItem->setY(0);
        m_pixmapItem->setPixmap(pixmap);

        foreach(QGraphicsPixmapItem *pixmapItem, m_pixmapItems) {
            if (pixmapItem != m_pixmapItem) {
                pixmapItem->setY( pixmapItem->y() - image->height() * imagesToJoinAndProcess );
            }
        }
    }

    m_currentJoinedImageRowIndex++;
}

} // namespace Z3D
