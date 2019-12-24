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

#include "zcameraacquisition_fwd.h"
#include "zcameraacquisition_global.h"

#include <QDebug>
#include <QObject>
#include <QPointer>
#include <QStringList>
#include <QVariant>

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraInterface : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString uuid READ uuid CONSTANT)
    Q_PROPERTY(bool running READ isRunning NOTIFY runningChanged)
    Q_PROPERTY(QStringList presets READ getPresets CONSTANT)

public:
    enum CameraAttributeType {
        CameraAttributeTypeUnknown = 0,
        CameraAttributeTypeBool,
        CameraAttributeTypeString,
        CameraAttributeTypeInt,
        CameraAttributeTypeLong,
        CameraAttributeTypeFloat,
        CameraAttributeTypeEnum,
        CameraAttributeTypeCommand
    };

    struct ZCameraAttribute {
        ZCameraAttribute()
            : id()
            , path()
            , label("UNKNOWN")
            , value("UNKNOWN")
            , readable(false)
            , writable(false)
            , type(CameraAttributeTypeUnknown)
            , maximumValue(std::numeric_limits<double>::max())
            , minimumValue(std::numeric_limits<double>::min())
            , enumValue(-1)
        {

        }

        QString id;
        QString path;
        QString label;
        QString description;
        QVariant value;
        bool readable;
        bool writable;
        CameraAttributeType type;
        double maximumValue;
        double minimumValue;
        int enumValue;
        QStringList enumNames;
    };

    explicit ZCameraInterface(QObject *parent = nullptr)
        : QObject(parent) {}

    virtual ~ZCameraInterface() {}

    virtual QString uuid() = 0;

    virtual int bufferSize() = 0;

signals:
    void newImageReceived(Z3D::ZCameraImagePtr image);

    void message(QString message);
    void warning(QString warningMessage);
    void error(QString errorMessage);

    void acquisitionStarted();
    void acquisitionStopped();

    void runningChanged();

    void attributeChanged(QString name, QVariant value);

public slots:
    virtual bool requestSnapshot() = 0;
    virtual Z3D::ZCameraImagePtr getSnapshot() = 0;

    /// acquisition control
    Q_INVOKABLE virtual bool startAcquisition() = 0;
    Q_INVOKABLE virtual bool stopAcquisition() = 0;
    virtual bool isRunning() = 0;

    /// camera attributes (settings)
    virtual QList<Z3D::ZCameraInterface::ZCameraAttribute> getAllAttributes() = 0;
    virtual bool setAttribute(const QString &name, const QVariant &value) = 0;
    virtual QVariant getAttribute(const QString &name) const = 0;

    virtual void showSettingsDialog() = 0;

    /// camera configuration
    virtual bool loadConfiguration(QString configFileName) = 0;

    /// presets
    virtual QStringList getPresets() = 0;
    Q_INVOKABLE virtual bool loadPresetConfiguration(QString presetName) = 0;

    virtual bool setBufferSize(int bufferSize) = 0;
};

} // namespace Z3D
