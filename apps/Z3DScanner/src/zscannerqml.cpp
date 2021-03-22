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

#include "ZCore/zsettingsitem.h"
#include "ZPointCloud/zpointcloudlistitem.h"
#include "ZPointCloud/zpointcloudlistmodel.h"
#include "ZStructuredLight/zpatternprojection.h"
#include "ZStructuredLight/zpatternprojectionprovider.h"
#include "ZStructuredLight/zstructuredlightsystem.h"
#include "ZStructuredLight/zstructuredlightsystemprovider.h"

#include <QDateTime>

ZScannerQML::ZScannerQML(const Z3D::ZStructuredLightSystemPtr &structuredLightSystem,
                         QObject *parent)
    : QObject(parent)
    , m_structuredLightSystem(structuredLightSystem)
    , m_patternProjectionSettings(new Z3D::ZSettingsItemModel(m_structuredLightSystem->patternProjection()->settings()))
    , m_structuredLightSystemSettings(new Z3D::ZSettingsItemModel(m_structuredLightSystem->settings()))
    , m_model(new Z3D::ZPointCloudListModel(this))
{
    QObject::connect(m_structuredLightSystem.get(), &Z3D::ZStructuredLightSystem::scanFinished,
                     this, &ZScannerQML::addNewScan);
}

Z3D::ZSettingsItemModel *ZScannerQML::patternProjectionSettings() const
{
    return m_patternProjectionSettings;
}

Z3D::ZSettingsItemModel *ZScannerQML::structuredLightSystemSettings() const
{
    return m_structuredLightSystemSettings;
}

Z3D::ZPointCloudListModel *ZScannerQML::model() const
{
    return m_model;
}

void ZScannerQML::scan()
{
    m_structuredLightSystem->start();
}

void ZScannerQML::addNewScan(const Z3D::ZPointCloudPtr &cloud)
{
    auto *pcItem = new Z3D::ZPointCloudListItem(cloud, QDateTime::currentDateTime().toString(Qt::ISODate));
    m_model->addPointCloud(pcItem);
}

#include "moc_zscannerqml.cpp"
#include "zscannerqml.moc"
