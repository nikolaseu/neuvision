#ifndef Z3D_CAMERAACQUISITION___ZIMAGEVIEWER_H
#define Z3D_CAMERAACQUISITION___ZIMAGEVIEWER_H

#include "zcameraacquisition_global.h"
#include "zcameraimage.h"

#include <QGraphicsPixmapItem>
#include <QGraphicsView>

//class QMenu;
class QSignalMapper;

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZImageViewer : public QGraphicsView
{
    Q_OBJECT

public:
    ZImageViewer(QWidget *parent = 0);
    ~ZImageViewer();

public slots:
    void updateImage(Z3D::ZImageGrayscale::Ptr image);
    void updateImage(cv::Mat image);
    void updateImage(QImage image);

    void setFitToWindow(const bool &enabled);
    void setDeleteOnClose(bool deleteOnClose);

protected slots:
    void changeColormap(int colormapId);
    void saveImage();

protected:
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void wheelEvent(QWheelEvent* event);
    virtual void resizeEvent(QResizeEvent* event);
    virtual void closeEvent(QCloseEvent *event);

protected:
    QMenu *m_contextMenu;
    QAction *m_fitToWindowAction;

    QMenu *m_colormapsMenu;
    QSignalMapper *m_colormapSignalMapper;
    int m_colormap;

    QGraphicsScene* m_scene;
    QGraphicsView* m_view;
    QGraphicsPixmapItem *m_pixmapItem;
    QImage m_image;

    double m_zoomFactor;
    bool m_fitToWindowEnabled;
    bool m_deleteOnClose;

    cv::Mat m_img;
    cv::Mat m_visibleImage;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION___ZIMAGEVIEWER_H
