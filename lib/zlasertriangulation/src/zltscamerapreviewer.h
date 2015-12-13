#ifndef Z3D_LASERTRIANGULATION___LTSCAMERAPREVIEWER_H
#define Z3D_LASERTRIANGULATION___LTSCAMERAPREVIEWER_H

#include "zlasertriangulation_global.h"
#include "zltscamera3d.h"

#include <QMainWindow>

namespace Z3D
{

namespace Ui {
class ZLTSCameraPreviewer;
}

class LTSProfilePlot;

class Z3D_LASERTRIANGULATION_SHARED_EXPORT LTSCameraPreviewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit LTSCameraPreviewer(Z3D::LTSCamera3D::Ptr camera, QWidget *parent = 0);
    ~LTSCameraPreviewer();

protected slots:
    void closeEvent(QCloseEvent *);

    void onNewImageReceived(Z3D::ZImageGrayscale::Ptr image);
    void onNewPointCloudReceived(Z3D::ZPointCloud::Ptr cloud);

    void onRangeMinimumChanged(double min);
    void onRangeMaximumChanged(double max);
private slots:
    void on_rangeMinSpinBox_valueChanged(double arg1);

    void on_rangeMaxSpinBox_valueChanged(double arg1);

    void on_actionPlayPause_triggered();

    void on_actionAutoSearchROI_triggered();

    void on_actionShowCameraSettings_triggered();

    void on_actionUnsharpMasking_triggered();

    void on_autoSetMinMaxButton_clicked();

    void on_scanViewDisplayModeComboBox_currentIndexChanged(int index);

private:
    Ui::ZLTSCameraPreviewer *ui;

    LTSProfilePlot *m_plot;
    QVector<QPointF> m_samples;

    Z3D::LTSCamera3D::Ptr m_camera;
};

} // namespace Z3D

#endif // Z3D_LASERTRIANGULATION___LTSCAMERAPREVIEWER_H
