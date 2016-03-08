#pragma once

#include "zstructuredlight_global.h"
#include "zcoreplugin.h"

namespace Z3D
{

class ZStructuredLightSystem;

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZStructuredLightSystemPlugin : public ZCorePlugin
{
    Q_OBJECT

public:
    virtual ~ZStructuredLightSystemPlugin() {}

    virtual QList<ZStructuredLightSystem*> getAll() = 0;
};

} // namespace Z3D

#define ZStructuredLightSystemPlugin_iid "z3d.zstructuredlight.zstructuredlightsystemplugin"

Q_DECLARE_INTERFACE(Z3D::ZStructuredLightSystemPlugin, ZStructuredLightSystemPlugin_iid)
