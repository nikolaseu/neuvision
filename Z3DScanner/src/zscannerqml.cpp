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

#include "zpatternprojection.h"
#include "zpatternprojectionprovider.h"
#include "zstructuredlightsystem.h"
#include "zstructuredlightsystemprovider.h"

#include <QWidget>

ZScannerQML::ZScannerQML(Z3D::ZStructuredLightSystemPtr structuredLightSystem, QObject *parent)
    : QObject(parent)
    , m_structuredLightSystem(structuredLightSystem)
{
    QObject::connect(m_structuredLightSystem.get(), &Z3D::ZStructuredLightSystem::scanFinished,
                     this, &ZScannerQML::setCloud);
}

Z3D::ZPointCloud *ZScannerQML::cloud() const
{
    return m_cloud.get();
}

void ZScannerQML::scan()
{
    m_structuredLightSystem->start();
}

//! FIXME delete this: use model for settings so we can display them in a generic/unified way in c++ and/or QML, just like camera settings
void ZScannerQML::openPatternsDialog() const
{
    QWidget *patternProjectionWidget = Z3D::ZPatternProjectionProvider::getConfigWidget(m_structuredLightSystem->patternProjection().get());
    patternProjectionWidget->show();
}

//! FIXME delete this: use model for settings so we can display them in a generic/unified way in c++ and/or QML, just like camera settings
void ZScannerQML::openStructuredLightSystemDialog() const
{
    QWidget *structuredLightSystemWidget = Z3D::ZStructuredLightSystemProvider::getConfigWidget(m_structuredLightSystem.get());
    structuredLightSystemWidget->show();
}

void ZScannerQML::setCloud(Z3D::ZPointCloudPtr cloud)
{
    if (m_cloud == cloud) {
        return;
    }

    m_cloud = cloud;
    emit cloudChanged(m_cloud.get());
}
