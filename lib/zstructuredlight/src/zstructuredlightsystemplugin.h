#pragma once

#include "zstructuredlight_global.h"
#include "zcoreplugin.h"

#include "zstructuredlightsystem.h"

QT_BEGIN_NAMESPACE
class QSettings;
QT_END_NAMESPACE

namespace Z3D
{

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZStructuredLightSystemPlugin : public ZCorePlugin
{
    Q_OBJECT

public:
    virtual ~ZStructuredLightSystemPlugin() {}

    virtual QList<QString> getAll() = 0;

    virtual Z3D::ZStructuredLightSystem::Ptr get(QSettings *settings) = 0;
};

} // namespace Z3D

#define ZStructuredLightSystemPlugin_iid "z3d.zstructuredlight.zstructuredlightsystemplugin"

Q_DECLARE_INTERFACE(Z3D::ZStructuredLightSystemPlugin, ZStructuredLightSystemPlugin_iid)
