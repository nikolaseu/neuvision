#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "src/sls/zsimplepointcloud.h"

#include <QMainWindow>
#include <QPointer>

#include <opencv2/core/core.hpp>

namespace Ui {
class MainWindow;
}

namespace Z3D {
class ZCloudView;
class ZPatternProjection;
class ZStructuredLightSystem;
}

class QSignalMapper;
//class QThread;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

signals:

private slots:
    void init();

    virtual void closeEvent(QCloseEvent *event);

    /// FIXME Z3D::ZCloudView *getCloudViewer();

    //
    void onPatternProjectionTypeChanged(int index);

    // each camera's submenu items
    void openCameraPreview(int camIndex);
    void openCameraSettingsDialog(int camIndex);
    void openCameraCalibrationWindow(int camIndex);
    void loadCameraCalibrationFile(int camIndex);
    void take3dCameraSnapshot(int camIndex);

    void onScanFinished(Z3D::ZSimplePointCloud::Ptr cloud);
    void getThreePhasePatternPhotos();

    void on_actionCloudViewer_triggered();
    void on_actionQuit_triggered();
    void on_actionShowSnapshot3D_triggered();

private:
    Ui::MainWindow *ui;

    QList<Z3D::ZPatternProjection *> m_patternProjectionList;
    Z3D::ZPatternProjection *m_currentPatternProjection;

    Z3D::ZStructuredLightSystem *m_structuredLightSystem;

    /// 3D view window (created on demand)
    /// FIXME QPointer<Z3D::ZCloudView> m_cloudViewer;

    QSignalMapper *m_previewSignalMapper;
    QSignalMapper *m_settingsSignalMapper;
    QSignalMapper *m_calibrateCameraSignalMapper;
    QSignalMapper *m_loadCalibrationSignalMapper;
    QSignalMapper *m_take3dSnapshotSignalMapper;
};

#endif // MAINWINDOW_H
