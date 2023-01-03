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

#include "ZCameraAcquisition/zcamerainterface_p.h"

#include <QtMultimedia/QCamera>
#include <QtMultimedia/QImageCapture>
#include <QtMultimedia/QMediaCaptureSession>

namespace Z3D
{

class ZQtCamera : public ZCameraBase
{
    Q_OBJECT

public:
    explicit ZQtCamera(QCamera *qcamera);
    ~ZQtCamera() override;

public slots:
    bool startAcquisition() override;
    bool stopAcquisition() override;

    QList<Z3D::ZCameraInterface::ZCameraAttribute> getAllAttributes() override;
    bool setAttribute(const QString &name, const QVariant &value) override;
    QVariant getAttribute(const QString &name) const override;

protected slots:
    bool setAttribute(const QString &name, const QVariant &value, bool notify) override;

    void onImageAvailable(int id, const QVideoFrame &buffer);
    void onCaptureError(int id, QImageCapture::Error error, const QString &message);
    void onReadyForCaptureChanged(bool ready);

private:
    QCamera *const m_camera;
    QMediaCaptureSession *const m_captureSession;
    QImageCapture *const m_imageCapture;

    bool m_isCapturing {false};
};

} // namespace Z3D
