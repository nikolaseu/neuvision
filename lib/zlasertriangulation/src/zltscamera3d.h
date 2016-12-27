/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

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
