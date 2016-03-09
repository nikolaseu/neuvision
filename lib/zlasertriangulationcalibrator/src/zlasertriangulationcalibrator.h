#pragma once

#include "zltscalibrationimagemodel.h"
#include "zcalibrationpatternfinder.h"

#include <Z3DCameraCalibration>

#include <pcl/visualization/pcl_visualizer.h>

#include "QVTKWidget.h"

#include <QFutureWatcher>
#include <QObject>
#include <QPointer>
#include <QPointF>
#include <QThread>

namespace Z3D
{

class ZLaserTriangulationCalibrator : public QObject
{
    Q_OBJECT

    Q_PROPERTY(Z3D::ZLTSCalibrationImageModel* imageModel    READ imageModel    WRITE setImageModel    NOTIFY imageModelChanged)
    //Q_PROPERTY(Z3D::ZCalibrationPatternFinder* patternFinder READ patternFinder WRITE setPatternFinder NOTIFY patternFinderChanged)
    Q_PROPERTY(Z3D::ZCameraCalibration* cameraCalibration READ cameraCalibration WRITE setCameraCalibration NOTIFY cameraCalibrationChanged)
    Q_PROPERTY(float progress READ progress NOTIFY progressChanged)

public:
    explicit ZLaserTriangulationCalibrator(QObject *parent = 0);
    ~ZLaserTriangulationCalibrator();

    Z3D::ZLTSCalibrationImageModel* imageModel() const;
    Z3D::ZCalibrationPatternFinder::Ptr patternFinder() const;
    Z3D::ZCameraCalibration* cameraCalibration() const;

    float progress() const;

    QWidget *getViewerWidget() const;

signals:
    void imageModelChanged(Z3D::ZLTSCalibrationImageModel* arg);
    void patternFinderChanged(Z3D::ZCalibrationPatternFinder::Ptr arg);
    void cameraCalibrationChanged(Z3D::ZCameraCalibration* arg);

    void progressChanged(float progress, QString message = QString());

    void calibrationChanged();

public slots:
    void setImageModel(Z3D::ZLTSCalibrationImageModel* imageModel);
    void setPatternFinder(ZCalibrationPatternFinder::Ptr patternFinder);
    void setCameraCalibration(Z3D::ZCameraCalibration* cameraCalibration);

    void setProgress(float progress, QString message = QString());
    void setProgressValue(int arg);
    void setProgressRange(int min, int max);

    Q_INVOKABLE void findCalibrationPattern();
    Q_INVOKABLE void calibrate();

    Q_INVOKABLE void saveResults(const QString &fileName);

protected:
    void calibrateFunctionImpl();

    Z3D::ZLTSCalibrationImageModel::WeakPtr m_imageModel;
    Z3D::ZCalibrationPatternFinder::Ptr m_patternFinder;
    Z3D::ZCameraCalibration::WeakPtr m_cameraCalibration;

    QFutureWatcher<void> m_patternFinderFutureWatcher;
    QFutureWatcher<void> m_calibrateFutureWatcher;

    float m_progress;
    int m_minProgressValue;
    int m_maxProgressValue;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_pclViewer;
    QVTKWidget *m_vtkWidget;
};

} // namespace Z3D
