#include "zimageviewer.h"

#include <QGLWidget>
#include <QMenu>
#include <QMouseEvent>
#include <QFileDialog>
#include <QDebug>
#include <QScrollBar>
#include <QSignalMapper>

#include <qmath.h>

namespace Z3D
{

ZImageViewer::ZImageViewer(QWidget *parent) :
    QGraphicsView(parent),
    m_colormap(-1), /// colormap < 0  -->  no colormap
    m_scene(new QGraphicsScene()),
    m_pixmapItem(new QGraphicsPixmapItem()),
    m_zoomFactor(0),
    m_fitToWindowEnabled(true),
    m_deleteOnClose(false)
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

    /// save image action
    QAction *saveImageAction = new QAction(tr("Save image as..."), m_contextMenu);
    QObject::connect(saveImageAction, SIGNAL(triggered()),
                     this, SLOT(saveImage()));
    m_contextMenu->addSeparator();
    m_contextMenu->addAction(saveImageAction);

    /// add pixmap item
    QPixmap image(":/empty.png");
    m_pixmapItem->setPixmap(image);
    m_scene->addItem(m_pixmapItem);
}

ZImageViewer::~ZImageViewer ()
{
    qDebug() << Q_FUNC_INFO;
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
        qreal scale = matrix().m11();
        /// get exponent -> log_2(scale) == log(x)/log(2)
        qreal exp = log(scale) /  log(qreal(2));
        m_zoomFactor = exp * qreal(10); /// este 10 tiene que ser el mismo que uso en wheelEvent!
    }
}

void ZImageViewer::setDeleteOnClose(bool deleteOnClose)
{
    m_deleteOnClose = deleteOnClose;
}

void ZImageViewer::changeColormap(int colormapId)
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
    if (m_img.cols)
        updateImage(m_img);
}

void ZImageViewer::saveImage()
{
    /// get file name
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save image as..."),
                                                    "image.png",
                                                    tr("Images (*.png *.jpg *.jpeg *.bmp)"));

    if (fileName.isEmpty() || fileName.isNull())
        return;

    /// save image as currently seen (w/colormap for example), not the original
    /// format=0 (use filename extension)
    /// quality=100 (maximum)
    if (!m_image.save(fileName /*, 0, 100*/)) {
        qCritical() << "unable to save image to file" << fileName;
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

        if (event->delta() > 0)
            m_zoomFactor++;
        else
            m_zoomFactor--;

        qreal scale = qPow(qreal(2), qreal(m_zoomFactor) / qreal(10)); /// este 10 tiene que ser el mismo que uso en setFitToWindow!

        QMatrix matrix;
        matrix.scale(scale, scale);
        setMatrix(matrix);

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

void ZImageViewer::updateImage(ZImageGrayscale::Ptr image)
{
    if (image)
        updateImage(image->cvMat());
    else
        updateImage(cv::Mat());
}

void ZImageViewer::updateImage(cv::Mat image)
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
    } else {
        qCritical() << "unsupported image";
        return;
    }

    /// last check, later we assume we have rgb 8 bit per channel!
    if (m_visibleImage.type() != CV_8UC3) {
        qDebug() << "converting image to CV_8UC3 from type:" << m_visibleImage.type();
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

void ZImageViewer::updateImage(QImage image)
{
    m_image = image;

    QPixmap pixmap = QPixmap::fromImage(m_image);

    m_pixmapItem->setPixmap(pixmap);
}

} // namespace Z3D
