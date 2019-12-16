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

#define CAMERA_MESSAGE(str) QString("[%1] %2").arg(uuid()).arg(str)
#define CAMERA_DEBUG(str)   qDebug()    << CAMERA_MESSAGE(str); emit message( CAMERA_MESSAGE(str) );
#define CAMERA_WARNING(str) qWarning()  << CAMERA_MESSAGE(str); emit warning( CAMERA_MESSAGE(str) );
#define CAMERA_ERROR(str)   qWarning()  << CAMERA_MESSAGE(str); emit error(   CAMERA_MESSAGE(str) );

#include "zcameraacquisition_global.h"
#include "zcamerainterface.h"

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraBase : public ZCameraInterface
{
    Q_OBJECT

public:
    explicit ZCameraBase(QObject *parent = nullptr);

    virtual QString uuid() override { return m_uuid; }

    virtual int bufferSize() override;

public slots:
    ///
    virtual bool requestSnapshot() override;
    virtual ZCameraImagePtr getSnapshot() override;

    /// acquisition control
    virtual bool startAcquisition() override;
    virtual bool stopAcquisition() override;
    virtual bool isRunning() override { return m_simultaneousCapturesCount > 0; }

    /// camera attributes (settings)
    virtual QList<ZCameraAttribute> getAllAttributes() override = 0;
    virtual bool setAttribute(const QString &name, const QVariant &value) override;
    virtual QVariant getAttribute(const QString &name) const override = 0;

    virtual void showSettingsDialog() override;

    /// camera configuration
    virtual bool loadConfiguration(QString configFileName) override;

    /// presets
    virtual QStringList getPresets() override;
    virtual bool loadPresetConfiguration(QString presetName) override;

    virtual bool setBufferSize(int bufferSize) override;

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify) = 0;

protected:
    QString m_uuid;

    QMap< QString, QList<ZCameraAttribute> > m_cameraPresets;

    ZCameraImagePtr *m_lastRetrievedImage;

    ZCameraImagePtr getNextBufferImage(int width, int height, int xOffset, int yOffset, int bytesPerPixel, void *externalBuffer = nullptr);

private:
    std::vector<ZCameraImagePtr> m_imagesBuffer;
    int m_currentImageBufferIndex;

    int m_simultaneousCapturesCount;

    /// camera settings
    QWidget *m_settingsWidget;
};

} // namespace Z3D
