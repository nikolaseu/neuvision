#ifndef ZSTEREOSTRUCTUREDLIGHTSYSTEM_H
#define ZSTEREOSTRUCTUREDLIGHTSYSTEM_H

#include "zstructuredlightsystem.h"
#include "zcalibratedcamera.h"

class StereoSystem;

namespace Z3D
{

class ZMultiCameraCalibratorWidget;

class ZStereoStructuredLightSystem : public ZStructuredLightSystem
{
    Q_OBJECT

public:
    explicit ZStereoStructuredLightSystem(QObject *parent = 0);
    ~ZStereoStructuredLightSystem();

    QWidget *getCalibrationWindow();

signals:

public slots:
    void setAcquisitionManager(ZCameraAcquisitionManager *acquisitionManager) override;

protected slots:
    void onPatternDecoded(Z3D::ZDecodedPattern::Ptr decodedPattern) override;

private:
    void init();
    void addCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameras);

    StereoSystem *m_stereoSystem;

    QList<Z3D::ZCalibratedCamera::Ptr> m_camList;

    double m_maxValidDistance = 0.002;

    bool m_debugSaveFringePoints;
    bool m_debugShowDecodedImages;
    bool m_debugShowFringes;

    /// Calibration window (created on demand)
    QPointer<Z3D::ZMultiCameraCalibratorWidget> m_calibrationWindow;
};

} // namespace Z3D

#endif // ZSTEREOSTRUCTUREDLIGHTSYSTEM_H
