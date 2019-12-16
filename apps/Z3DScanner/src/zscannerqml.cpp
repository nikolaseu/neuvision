//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2018 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "zscannerqml.h"

#include "ZStructuredLight/zpatternprojection.h"
#include "ZStructuredLight/zpatternprojectionprovider.h"
#include "ZStructuredLight/zstructuredlightsystem.h"
#include "ZStructuredLight/zstructuredlightsystemprovider.h"

#include "ZCore/zsettingsitem.h"

#include <QWidget>

ZScannerQML::ZScannerQML(Z3D::ZStructuredLightSystemPtr structuredLightSystem, QObject *parent)
    : QObject(parent)
    , m_structuredLightSystem(structuredLightSystem)
    , m_patternProjectionSettings(new Z3D::ZSettingsItemModel(m_structuredLightSystem->patternProjection()->settings()))
    , m_structuredLightSystemSettings(new Z3D::ZSettingsItemModel(m_structuredLightSystem->settings()))
{
    QObject::connect(m_structuredLightSystem.get(), &Z3D::ZStructuredLightSystem::scanFinished,
                     this, &ZScannerQML::setCloud);
}

Z3D::ZSettingsItemModel *ZScannerQML::patternProjectionSettings() const
{
    return m_patternProjectionSettings;
}

Z3D::ZSettingsItemModel *ZScannerQML::structuredLightSystemSettings() const
{
    return m_structuredLightSystemSettings;
}

Z3D::ZPointCloud *ZScannerQML::cloud() const
{
    return m_cloud.get();
}

void ZScannerQML::scan()
{
    m_structuredLightSystem->start();
}

void ZScannerQML::setCloud(Z3D::ZPointCloudPtr cloud)
{
    if (m_cloud == cloud) {
        return;
    }

    m_cloud = cloud;
    emit cloudChanged(m_cloud.get());
}
