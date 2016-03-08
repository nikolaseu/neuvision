#pragma once

#include "zcalibrationimagemodel.h"
#include "zcalibrationpatternfinder.h"

#include "zcameracalibration.h"

#include <QFutureWatcher>
#include <QObject>

namespace Z3D
{

class ZCameraCalibrator;

class ZCameraCalibratorWorker : public QObject
{
    Q_OBJECT

public:
    explicit ZCameraCalibratorWorker(QObject *parent = 0);
    ~ZCameraCalibratorWorker();

    Z3D::ZCalibrationImageModel* imageModel();
    Z3D::ZCalibrationPatternFinder::Ptr patternFinder();
    ZCameraCalibrator *cameraCalibrator();

    float progress() const;

signals:
    void imageModelChanged();
    void patternFinderChanged();

    void progressChanged(float progress, QString message = QString());

    void calibrationFailed(QString message);
    void calibrationChanged(Z3D::ZCameraCalibration::Ptr calibration);

public slots:
    void setImageModel(Z3D::ZCalibrationImageModel* imageModel);
    void setPatternFinder(ZCalibrationPatternFinder::Ptr patternFinder);
    void setCameraCalibrator(ZCameraCalibrator *cameraCalibrator);

    void setProgress(float progress, QString message = QString());
    void setProgressValue(int arg);
    void setProgressRange(int min, int max);

    void findCalibrationPattern();
    void calibrate();

protected:
    void calibrateFunctionImpl();

    Z3D::ZCalibrationImageModel::WeakPtr m_imageModel;
    Z3D::ZCalibrationPatternFinder::Ptr m_patternFinder;
    ZCameraCalibrator *m_cameraCalibrator;

    QFutureWatcher<void> m_patternFinderFutureWatcher;
    QFutureWatcher<void> m_calibrateFutureWatcher;

    float m_progress;
    int m_minProgressValue;
    int m_maxProgressValue;
};

} // namespace Z3D
