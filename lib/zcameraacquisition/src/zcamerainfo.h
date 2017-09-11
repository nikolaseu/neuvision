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

#include "zcameraacquisition_global.h"
#include "zcamerainterface.h"

#include <QObject>
#include <QVariantMap>

namespace Z3D
{

/// FW declaration to avoid cyclic includes
class ZCameraPluginInterface;

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraInfo : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString name          READ name       CONSTANT)
    Q_PROPERTY(QString pluginName    READ pluginName CONSTANT)
    Q_PROPERTY(QVariantMap extraData READ extraData  CONSTANT)

public:
    ZCameraInfo(ZCameraPluginInterface *plugin, QString name, QVariantMap extraData, QObject *parent = nullptr);
    ~ZCameraInfo();

    QString name() const;
    QString pluginName() const;
    QVariantMap extraData() const;

    ZCameraInterface::Ptr getCamera() const;

private:
    ZCameraPluginInterface* m_plugin;

    QString m_name;
    QVariantMap m_extraData;
};

} // namespace Z3D
