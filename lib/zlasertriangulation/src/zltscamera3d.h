#pragma once

#include "zlasertriangulation_global.h"
#include "zcamera3dinterface.h"
#include "zltscamera3dcalibrationinterface.h"

#include <Z3DCameraAcquisition>
#include <Z3DPointCloud>

namespace Z3D
{

class Z3D_LASERTRIANGULATION_SHARED_EXPORT LTSCamera3D : public Camera3DInterface
{
    Q_OBJECT

    Q_PROPERTY(int profileDivisor READ profileDivisor WRITE setProfileDivisor NOTIFY profileDivisorChanged)

public:
    typedef QSharedPointer<Z3D::LTSCamera3D> Ptr;
    typedef QPointer<Z3D::LTSCamera3D> WeakPtr;

    enum AcquisitionMode {
        UnknownAcquisitionMode = 0x00,
        ImageAcquisitionMode   = 0x01,
        ProfileAcquisitionMode = 0x02,
        DualAcquisitionMode    = 0x10
    };

    explicit LTSCamera3D(ZCalibratedCamera::Ptr camera,
                         LTSCamera3DCalibrationInterface::Ptr calibration = LTSCamera3DCalibrationInterface::Ptr(),
                         QObject *parent = 0);

    ~LTSCamera3D();

    int profileDivisor() const;

signals:
    void profileDivisorChanged(int arg);

public slots:
    void startAcquisition();
    void stopAcquisition();

    int acquisitionMode() const;
    bool setAcquisitionMode(int mode);

    void setProfileDivisor(int arg);

protected slots:
    void onNewImageReceived(Z3D::ZImageGrayscale::Ptr image);

    void processProfiles(Z3D::ZImageGrayscale::Ptr image);

    void getRealWorldPositionForPixel(int x, int y, float *realX, float *realY, float *realZ);

protected:
    LTSCamera3DCalibrationInterface::Ptr m_ltsCalibration;

    /*AcquisitionMode*/
    int m_acquisitionMode;

    std::vector<Z3D::ZPointCloud::Ptr> m_cloudsBuffer;

    int m_cloudsBufferSize;

    int m_currentCloudIndex;

    //!
    //! \brief pointStep is the factor used to reduce the number of points to display
    //!
    int m_pointsStep;

    float m_zStep;

private:
    int m_profileDivisor;

};

} // namespace Z3D
