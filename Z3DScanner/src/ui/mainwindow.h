#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "src/sls/binarypatternprojection.h"

#include <Z3DCalibratedCamera>

#include <opencv2/core/core.hpp>

namespace Ui {
class MainWindow;
}

namespace Z3D {
class ZCloudView;
class ZMultiCameraCalibratorWidget;
}

class CalibrationWindow;
class StereoSystem;

class QSignalMapper;
//class QThread;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QList<Z3D::ZCalibratedCamera::Ptr> getCameras() { return m_camList; }

signals:
    void cameraListChanged(QList<Z3D::ZCalibratedCamera::Ptr> cameras);

private slots:
    void init();

    virtual void closeEvent(QCloseEvent *event);

    Z3D::ZCloudView *getCloudViewer();
    QWidget *getCalibrationWindow();

    //
    void onPatternProjectionTypeChanged(int index);

    // add cameras to be used
    void addCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameras);

    // each camera's submenu items
    void openCameraPreview(int camIndex);
    void openCameraSettingsDialog(int camIndex);
    void openCameraCalibrationWindow(int camIndex);
    void loadCameraCalibrationFile(int camIndex);
    void take3dCameraSnapshot(int camIndex);

    void onPatternDecoded(Z3D::DecodedPattern::Ptr pattern);
    void getThreePhasePatternPhotos();

    void on_actionCloudViewer_triggered();
    void on_actionQuit_triggered();
    void on_actionCalibrateSystem_triggered();
    void on_actionShowSnapshot3D_triggered();

private:
    Ui::MainWindow *ui;

    QList<BinaryPatternProjection *> m_patternProjectionList;
    BinaryPatternProjection *m_currentPatternProjection;

    StereoSystem *m_stereoSystem;

    /// 3D view window (created on demand)
    QPointer<Z3D::ZCloudView> m_cloudViewer;

    /// Calibration window (created on demand)
    //CalibrationWindow *m_calibrationWindow;
    QPointer<Z3D::ZMultiCameraCalibratorWidget> m_calibrationWindow;

    QList<Z3D::ZCalibratedCamera::Ptr> m_camList;

    QSignalMapper *m_previewSignalMapper;
    QSignalMapper *m_settingsSignalMapper;
    QSignalMapper *m_calibrateCameraSignalMapper;
    QSignalMapper *m_loadCalibrationSignalMapper;
    QSignalMapper *m_take3dSnapshotSignalMapper;
};

#endif // MAINWINDOW_H
