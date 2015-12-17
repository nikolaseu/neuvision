#ifndef Z3D_STRUCTUREDLIGHT___ZSTEREOSTRUCTUREDLIGHTSYSTEM_H
#define Z3D_STRUCTUREDLIGHT___ZSTEREOSTRUCTUREDLIGHTSYSTEM_H

#include "zstructuredlightsystem.h"
#include "zcalibratedcamera.h"

namespace Z3D
{

class ZMultiCameraCalibratorWidget;
class ZStereoSLSConfigWidget;
class ZStereoSystemImpl;

class ZStereoSLS : public ZStructuredLightSystem
{
    Q_OBJECT

    Q_PROPERTY(double maxValidDistance READ maxValidDistance WRITE setMaxValidDistance NOTIFY maxValidDistanceChanged)
    Q_PROPERTY(bool debugSaveFringePoints READ debugSaveFringePoints WRITE setDebugSaveFringePoints NOTIFY debugSaveFringePointsChanged)
    Q_PROPERTY(bool debugShowDecodedImages READ debugShowDecodedImages WRITE setDebugShowDecodedImages NOTIFY debugShowDecodedImagesChanged)
    Q_PROPERTY(bool debugShowFringes READ debugShowFringes WRITE setDebugShowFringes NOTIFY debugShowFringesChanged)

public:
    explicit ZStereoSLS(QObject *parent = 0);
    ~ZStereoSLS();

    virtual QString displayName() override;

    QWidget *getCalibrationWindow();

    Z3D::ZCalibratedCamera::Ptr leftCamera() const;
    Z3D::ZCalibratedCamera::Ptr rightCamera() const;

    double maxValidDistance() const;
    bool debugSaveFringePoints() const;
    bool debugShowDecodedImages() const;
    bool debugShowFringes() const;

signals:
    void maxValidDistanceChanged(double maxValidDistance);
    void debugSaveFringePointsChanged(bool debugSaveFringePoints);
    void debugShowDecodedImagesChanged(bool debugShowDecodedImages);
    void debugShowFringesChanged(bool debugShowFringes);

public slots:
    void setMaxValidDistance(double maxValidDistance);
    void setDebugSaveFringePoints(bool debugSaveFringePoints);
    void setDebugShowDecodedImages(bool debugShowDecodedImages);
    void setDebugShowFringes(bool debugShowFringes);

    // ZStructuredLightSystem interface
    virtual QWidget *configWidget() override;

protected slots:
    void onPatternsDecoded(std::vector<Z3D::ZDecodedPattern::Ptr> decodedPatterns) override;

private:
    void init();
    void addCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameras);

    void setLeftCamera(Z3D::ZCalibratedCamera::Ptr camera);
    void setRightCamera(Z3D::ZCalibratedCamera::Ptr camera);

    ZStereoSystemImpl *m_stereoSystem;

    std::vector<Z3D::ZCalibratedCamera::Ptr> m_cameras;

    double m_maxValidDistance;
    bool m_debugSaveFringePoints;
    bool m_debugShowDecodedImages;
    bool m_debugShowFringes;

    /// Calibration window (created on demand)
    QPointer<Z3D::ZMultiCameraCalibratorWidget> m_calibrationWindow;

    /// Config widget (created on demand)
    ZStereoSLSConfigWidget *m_configWidget;
};

} // namespace Z3D

#endif // Z3D_STRUCTUREDLIGHT___ZSTEREOSTRUCTUREDLIGHTSYSTEM_H
