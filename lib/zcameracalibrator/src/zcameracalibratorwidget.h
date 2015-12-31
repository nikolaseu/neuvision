#ifndef Z3D_CAMERACALIBRATOR___ZCAMERACALIBRATORWIDGET_H
#define Z3D_CAMERACALIBRATOR___ZCAMERACALIBRATORWIDGET_H

#include "zcameracalibrator_global.h"

#include "zcalibrationpatternfinder.h"

#include <Z3DCalibratedCamera>

#include <QWidget>
#include <QModelIndex>

class QProgressBar;
class QThread;

class ObjectController;

namespace Z3D
{

namespace Ui {
class ZCameraCalibratorWidget;
}

class ZCalibrationDistortionPlot;
class ZCalibrationImageModel;
class ZCameraCalibratorWorker;
class ZCameraCalibrator;

class Z3D_CAMERACALIBRATOR_SHARED_EXPORT ZCameraCalibratorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ZCameraCalibratorWidget(Z3D::ZCalibratedCamera::Ptr camera = Z3D::ZCalibratedCamera::Ptr(0), QWidget *parent = 0);
    ~ZCameraCalibratorWidget();

private slots:
    virtual void closeEvent(QCloseEvent *event);

    void updateWindowTitle();

    void setCurrentView(int newView);

    void setImagesListVisible(bool visible);

    void onCurrentSelectionChanged(QModelIndex current, QModelIndex previous);

    void onCameraModelTypeChanged(int index);
    void onCalibrationPatternTypeChanged(int index);

    void onProgressChanged(float progress, QString message);
    void onCalibrationChanged(Z3D::ZCameraCalibration::Ptr newCalibration);

    void loadSession();
    void newSession();

    void on_calibrationPatternViewButton_clicked();
    void on_imageViewPageButton_clicked();
    void on_cameraViewPageButton_clicked();
    void on_calibrationPageButton_clicked();
    void on_resultsPageButton_clicked();

    void on_runCalibrationButton_clicked();

    void on_imageViewTypeComboBox_currentIndexChanged(int index);

    void on_saveCameraImageButton_clicked();

    void on_cameraSettingsButton_clicked();

    void on_saveCalibrationToFileButton_clicked();

    void on_updateCameraCalibrationButton_clicked();

private:
    Ui::ZCameraCalibratorWidget *ui;
    QProgressBar *m_statusProgressBar;
    ZCalibrationDistortionPlot *m_distortionPlot;

    ZCalibrationImageModel *m_model;
    QString m_sessionFolder;

    QList<ZCalibrationPatternFinder::Ptr> m_patternFinderList;
    QList<ZCameraCalibrator *> m_cameraCalibratorList;

    ZCameraCalibratorWorker *m_calibratorWorker;

    Z3D::ZCalibratedCamera::Ptr m_camera;

    Z3D::ZCameraCalibration::Ptr m_currentCalibration;

    ObjectController *m_calibrationParamsController;

    QThread *m_workerThread;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZCAMERACALIBRATORWINDOW_H
