#ifndef Z3D_CAMERACALIBRATOR___ZMULTICAMERACALIBRATORWORKER_H
#define Z3D_CAMERACALIBRATOR___ZMULTICAMERACALIBRATORWORKER_H

#include "zmulticalibrationimagemodel.h"
#include "zcalibrationpatternfinder.h"

#include "zcameracalibration.h"

#include <QFutureWatcher>
#include <QObject>

namespace Z3D
{

class ZMultiCameraCalibrator;

class ZMultiCameraCalibratorWorker : public QObject
{
    Q_OBJECT

public:
    explicit ZMultiCameraCalibratorWorker(QObject *parent = 0);
    ~ZMultiCameraCalibratorWorker();

    Z3D::ZMultiCalibrationImageModel* imageModel();
    ZCalibrationPatternFinder::Ptr patternFinder();
    Z3D::ZMultiCameraCalibrator *cameraCalibrator();

    float progress() const;

signals:
    void imageModelChanged();
    void patternFinderChanged();

    void progressChanged(float progress, QString message = QString());

    void calibrationFailed(QString message);
    void calibrationChanged(std::vector<Z3D::ZCameraCalibration::Ptr> calibrations);

public slots:
    void setImageModel(Z3D::ZMultiCalibrationImageModel* imageModel);
    void setPatternFinder(Z3D::ZCalibrationPatternFinder::Ptr patternFinder);
    void setCameraCalibrator(Z3D::ZMultiCameraCalibrator *cameraCalibrator);

    void setProgress(float progress, QString message = QString());
    void setProgressValue(int arg);
    void setProgressRange(int min, int max);

    void findCalibrationPattern();
    void calibrate(std::vector<Z3D::ZCameraCalibration::Ptr> currentCalibrations);

protected:
    void calibrateFunctionImpl(std::vector<ZCameraCalibration::Ptr> currentCalibrations);

    Z3D::ZMultiCalibrationImageModel::WeakPtr m_imageModel;
    Z3D::ZCalibrationPatternFinder::Ptr m_patternFinder;
    Z3D::ZMultiCameraCalibrator *m_cameraCalibrator;

    QFutureWatcher<void> m_patternFinderFutureWatcher;
    QFutureWatcher<void> m_calibrateFutureWatcher;

    float m_progress;
    int m_minProgressValue;
    int m_maxProgressValue;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZMULTICAMERACALIBRATORWORKER_H
