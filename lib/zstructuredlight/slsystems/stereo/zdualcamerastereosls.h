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

#include "zcalibratedcamera.h"
#include "zstereosls.h"

namespace Z3D
{

class ZDualCameraStereoSLSConfigWidget;

class ZDualCameraStereoSLS : public ZStereoSLS
{
    Q_OBJECT

public:
    explicit ZDualCameraStereoSLS(QObject *parent = nullptr);
    ~ZDualCameraStereoSLS();

    Z3D::ZCalibratedCamera::Ptr leftCamera() const;
    Z3D::ZCalibratedCamera::Ptr rightCamera() const;

private slots:
    void addCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameras);

    void setLeftCamera(Z3D::ZCalibratedCamera::Ptr camera);
    void setRightCamera(Z3D::ZCalibratedCamera::Ptr camera);

private:
    std::vector<Z3D::ZCalibratedCamera::Ptr> m_cameras;

    // ZStructuredLightSystem interface
public:
    virtual QString id() const override;
    virtual QString displayName() const override;

    virtual void init(QSettings *settings) override;

protected slots:
    virtual void onPatternProjected(ZProjectedPattern::Ptr pattern) override;
    virtual void onPatternsDecoded(std::vector<ZDecodedPattern::Ptr> patterns) override;
};

} // namespace Z3D
