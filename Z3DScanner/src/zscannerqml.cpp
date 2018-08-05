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
