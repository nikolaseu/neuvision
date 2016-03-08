#pragma once

#include "zstructuredlight_global.h"
#include "zcoreplugin.h"

namespace Z3D
{

class ZPatternProjection;

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZPatternProjectionPlugin : public ZCorePlugin
{
    Q_OBJECT

public:
    virtual ~ZPatternProjectionPlugin() {}

    virtual QList<ZPatternProjection*> getAll() = 0;
};

} // namespace Z3D

#define ZPatternProjectionPlugin_iid "z3d.zstructuredlight.zpatternprojectionplugin"

Q_DECLARE_INTERFACE(Z3D::ZPatternProjectionPlugin, ZPatternProjectionPlugin_iid)
