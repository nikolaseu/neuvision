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

#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>

#include <vector>

namespace Z3D
{

class PylonCamera : public ZCameraBase
{
    Q_OBJECT

    class BaslerImageHandler : public Pylon::CImageEventHandler
    {
    public:
        explicit BaslerImageHandler(PylonCamera *camera);

        virtual void OnImageGrabbed(Pylon::CInstantCamera& camera, const Pylon::CGrabResultPtr& grabResult) override;
    private:
        PylonCamera *m_camera;
    };

public:
    explicit PylonCamera(Pylon::IPylonDevice* device, QObject *parent = nullptr);
    ~PylonCamera();

public slots:
    virtual bool startAcquisition();
    virtual bool stopAcquisition();

    virtual QList<ZCameraAttribute> getAllAttributes();
    virtual QVariant getAttribute(const QString &name) const;

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify);

private:
    Pylon::CBaslerUsbInstantCamera m_camera;
    BaslerImageHandler* m_image_handler;

    std::vector<ZCameraImagePtr> m_imagesBuffer;
    int m_currentImageBufferIndex;

    int m_acquisitionCounter;
    int m_currentAcquisitionCounter;
};

} // namespace Z3D
