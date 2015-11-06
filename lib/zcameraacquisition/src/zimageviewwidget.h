#ifndef Z3D_CAMERAACQUISITION___ZIMAGEVIEWWIDGET_H
#define Z3D_CAMERAACQUISITION___ZIMAGEVIEWWIDGET_H

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

#endif // Z3D_CAMERAACQUISITION___ZIMAGEVIEWWIDGET_H
