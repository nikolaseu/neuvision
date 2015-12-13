#ifndef Z3D_CAMERACALIBRATOR___ZMULTICAMERACALIBRATORWINDOW_H
#define Z3D_CAMERACALIBRATOR___ZMULTICAMERACALIBRATORWINDOW_H

#include "zcameracalibrator_global.h"

#include "zcalibrationpatternfinder.h"

#include <Z3DCalibratedCamera>

#include <QWidget>
#include <QModelIndex>

class QProgressBar;

namespace Z3D
{

namespace Ui {
class ZMultiCameraCalibratorWidget;
}

class ZCalibrationDistortionPlot;
class ZImageViewer;
class ZCalibrationImageViewer;
class ZMultiCameraCalibratorWorker;
class ZMultiCalibrationImageModel;
class ZMultiCameraCalibrator;

class Z3D_CAMERACALIBRATOR_SHARED_EXPORT ZMultiCameraCalibratorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ZMultiCameraCalibratorWidget(std::vector<ZCalibratedCamera::Ptr> cameras, QWidget *parent = 0);
    ~ZMultiCameraCalibratorWidget();

private slots:
    virtual void closeEvent(QCloseEvent *event);

    void updateWindowTitle();

    void setCurrentView(int newView);

    void onCurrentSelectionChanged(QModelIndex current, QModelIndex previous);

    void onCameraModelTypeChanged(int index);
    void onCalibrationPatternTypeChanged(int index);

    void onProgressChanged(float progress, QString message);
    void onCalibrationChanged(std::vector<Z3D::ZCameraCalibration::Ptr> newCalibrations);

    void loadSession();
    void newSession();

    void on_imageViewPageButton_clicked();

    void on_cameraViewPageButton_clicked();

    void on_calibrationPageButton_clicked();

    void on_runCalibrationButton_clicked();

    void on_imageViewTypeComboBox_currentIndexChanged(int index);

    void on_calibrationPatternViewButton_clicked();

    void on_saveCameraImageButton_clicked();

    void on_cameraSettingsButton_clicked();

private:
    Ui::ZMultiCameraCalibratorWidget *ui;
    QProgressBar *m_statusProgressBar;

    Z3D::ZMultiCalibrationImageModel *m_model;
    QString m_sessionFolder;

    QList<Z3D::ZCalibrationPatternFinder::Ptr> m_patternFinderList;
    QList<Z3D::ZMultiCameraCalibrator *> m_cameraCalibratorList;

    Z3D::ZMultiCameraCalibratorWorker *m_calibratorWorker;

    std::vector<Z3D::ZCalibratedCamera::Ptr> m_cameras;

    std::vector<Z3D::ZImageViewer *> m_cameraImageViewer;
    std::vector<Z3D::ZCalibrationImageViewer *> m_imageViewer;

    QThread *m_workerThread;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZMULTICAMERACALIBRATORWINDOW_H
