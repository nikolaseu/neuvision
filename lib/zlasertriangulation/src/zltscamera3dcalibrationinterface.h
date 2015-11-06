#ifndef Z3D_LASERTRIANGULATION___LTSCAMERA3DCALIBRATIONINTERFACE_H
#define Z3D_LASERTRIANGULATION___LTSCAMERA3DCALIBRATIONINTERFACE_H

#include <QSharedPointer>
#include <QString>

namespace Z3D
{

class LTSCamera3DCalibrationInterface
{

public:
    typedef QSharedPointer<Z3D::LTSCamera3DCalibrationInterface> Ptr;

    virtual ~LTSCamera3DCalibrationInterface() {}

    virtual void getCalibratedPointForPixel(int x, int y, float *calibratedX, float *calibratedY, float *calibratedZ) = 0;

    virtual bool loadCalibration(const QString &fileName) = 0;
};

} // namespace Z3D

#endif // Z3D_LASERTRIANGULATION___LTSCAMERA3DCALIBRATIONINTERFACE_H
