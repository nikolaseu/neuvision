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

#ifndef Z3D_CAMERAACQUISITION_PLUGIN___FLYCAPTURE2CAMERA_H
#define Z3D_CAMERAACQUISITION_PLUGIN___FLYCAPTURE2CAMERA_H

#include "zcamerainterface_p.h"

#include "FlyCapture2.h"
#include "FlyCapture2Gui.h"

namespace Z3D
{

class ZFlyCapture2Camera : public ZCameraBase
{
    Q_OBJECT

public:
    static QList<ZCameraInterface::Ptr> getConnectedCameras();
    static ZCameraInterface::Ptr getCameraByName(QString name);

    ~ZFlyCapture2Camera();
    
signals:
    
public slots:
    void showSettingsDialog();

protected:
    explicit ZFlyCapture2Camera(FlyCapture2::Camera *cameraHandle);

private:
    static void imageEventCallback(FlyCapture2::Image* pImage, const void *pCallbackData );
    void imageEventCallbackInternal(FlyCapture2::Image* pImage);

    FlyCapture2::Camera* m_pCamera;
    FlyCapture2::CameraInfo m_cameraInfo;
    FlyCapture2::Image m_rawImage;
    FlyCapture2::Image m_processedImage;

    FlyCapture2::CameraControlDlg m_camCtlDlg;

    long m_currentBufferNumber;


    // ZCameraBase interface
public slots:
    virtual QList<ZCameraAttribute> getAllAttributes();
    virtual QVariant getAttribute(const QString &name) const;

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify);

    // ZCameraInterface interface
public:
    virtual bool startAcquisition();
    virtual bool stopAcquisition();
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___FLYCAPTURE2CAMERA_H
