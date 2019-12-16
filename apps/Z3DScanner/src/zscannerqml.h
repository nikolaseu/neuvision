/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2018 Nicolas Ulrich <nikolaseu@gmail.com>
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

#include "ZCore/zcore_fwd.h"
#include "ZPointCloud/zpointcloud_fwd.h"
#include "ZStructuredLight/zstructuredlight_fwd.h"

#include <QObject>

class ZScannerQML : public QObject
{
    Q_OBJECT

    Q_PROPERTY(Z3D::ZSettingsItemModel* patternProjectionSettings READ patternProjectionSettings CONSTANT)
    Q_PROPERTY(Z3D::ZSettingsItemModel* structuredLightSystemSettings READ structuredLightSystemSettings CONSTANT)
    Q_PROPERTY(Z3D::ZPointCloud* cloud READ cloud NOTIFY cloudChanged)

public:
    explicit ZScannerQML(Z3D::ZStructuredLightSystemPtr structuredLightSystem, QObject *parent = nullptr);

    Z3D::ZSettingsItemModel* patternProjectionSettings() const;
    Z3D::ZSettingsItemModel* structuredLightSystemSettings() const;

    Z3D::ZPointCloud* cloud() const;

signals:
    void cloudChanged(Z3D::ZPointCloud* cloud);

public slots:
    void scan();

private slots:
    void setCloud(Z3D::ZPointCloudPtr cloud);

private:
    const Z3D::ZStructuredLightSystemPtr m_structuredLightSystem;
    Z3D::ZSettingsItemModel* m_patternProjectionSettings;
    Z3D::ZSettingsItemModel* m_structuredLightSystemSettings;

    Z3D::ZPointCloudPtr m_cloud;
};
