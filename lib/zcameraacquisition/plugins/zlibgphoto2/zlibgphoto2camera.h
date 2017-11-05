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

#include "zcamerainterface_p.h"

#include <gphoto2/gphoto2-camera.h>

namespace Z3D
{

class ZLibGPhoto2Camera : public ZCameraBase
{
    Q_OBJECT

public:
    explicit ZLibGPhoto2Camera(GPContext *context, Camera *camera, QObject *parent = 0);
    ~ZLibGPhoto2Camera();

signals:

public slots:
    /// acquisition control
    bool startAcquisition() override;
    bool stopAcquisition() override;

    ZCameraImagePtr getSnapshot() override;

    /// camera attributes (settings)
    QList<ZCameraAttribute> getAllAttributes() override;
    QVariant getAttribute(const QString &id) const override;

protected slots:
    bool setAttribute(const QString &id, const QVariant &value, bool notify) override;

    void grabLoop();

private:
    bool m_stopThreadRequested;
    QMutex m_mutex;

    /// gPhoto2
    GPContext *context;
    Camera *cam;
};

} // namespace Z3D
