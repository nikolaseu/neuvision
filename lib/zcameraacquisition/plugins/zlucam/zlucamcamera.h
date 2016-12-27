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

#ifndef Z3D_CAMERAACQUISITION_PLUGIN___LUCAMCAMERA_H
#define Z3D_CAMERAACQUISITION_PLUGIN___LUCAMCAMERA_H

#include "zcamerainterface_p.h"

#include <windows.h> // lucamapi.h needs it
#include "lucamapi.h"

#include <vector>

namespace Z3D
{

class LuCamCamera : public ZCameraBase
{
    Q_OBJECT

public:
    static QList<ZCameraInterface::Ptr> getConnectedCameras();
    static ZCameraInterface::Ptr getCameraByName(QString name);

    explicit LuCamCamera(HANDLE cameraHandle, QObject *parent = 0);
    ~LuCamCamera();

public slots:
    virtual bool startAcquisition();
    virtual bool stopAcquisition();

    virtual QList<ZCameraAttribute> getAllAttributes();
    virtual QVariant getAttribute(const QString &name) const;

    virtual void showSettingsDialog();

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify);

private:
    static void __stdcall imageEventCallback(void *pCallbackData, BYTE *data, ULONG dataLenght);
    void imageEventCallbackInternal(BYTE *data, ULONG dataLenght);

    LONG m_snapshotCallbackId;
    LONG m_streamingCallbackId;

    HANDLE m_cameraHandle;

    LUCAM_FRAME_FORMAT m_frameFormat;

    LUCAM_SNAPSHOT m_snapshotSettings;

    LUCAM_VERSION m_lucamVersion;

    std::vector<ZImageGrayscale::Ptr> m_imagesBuffer;
    int m_currentImageBufferIndex;

    int m_acquisitionCounter;
    int m_currentAcquisitionCounter;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___LUCAMCAMERA_H
