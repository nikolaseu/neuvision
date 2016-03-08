#pragma once

#include "zlasertriangulationcalibrator_global.h"

#include "zcalibrationpatternfinder.h"

#include <Z3DCalibratedCamera>

#include <QMainWindow>
#include <QModelIndex>

namespace Ui {
class ZLaserTriangulationCalibratorWindow;
}

class QProgressBar;

namespace Z3D
{

class ZLTSCalibrationImageModel;
class ZLaserTriangulationCalibrator;

class Z3D_LASERTRIANGULATIONCALIBRATOR_SHARED_EXPORT ZLaserTriangulationCalibratorWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ZLaserTriangulationCalibratorWindow(ZCalibratedCamera::WeakPtr camera3D, QWidget *parent = 0);
    ~ZLaserTriangulationCalibratorWindow();

private slots:
    virtual void closeEvent(QCloseEvent *event);

    void onCurrentSelectionChanged(QModelIndex current, QModelIndex previous);
    void onCalibrationPatternTypeChanged(int index);

    void onProgressChanged(float progress, QString message);
    void onCalibrationChanged();

    void on_pushButton_clicked();

    void on_imageViewPageButton_clicked();

    void on_cameraViewPageButton_clicked();

    void on_calibrationPageButton_clicked();

    void on_runCalibrationButton_clicked();

    void on_imageViewTypeComboBox_currentIndexChanged(int index);

    void on_calibrationPatternViewButton_clicked();

private:
    Ui::ZLaserTriangulationCalibratorWindow *ui;
    QProgressBar *m_statusProgressBar;

    ZCalibratedCamera::WeakPtr m_camera;
    //Z3D::LTSCamera3D::WeakPtr m_camera3D;

    ZLTSCalibrationImageModel *m_model;

    QList<ZCalibrationPatternFinder::Ptr> m_patternFinderList;
    ZCalibrationPatternFinder::Ptr m_currentPatternFinder;

    ZLaserTriangulationCalibrator *m_ltsCalibrator;
};

} // namespace Z3D
