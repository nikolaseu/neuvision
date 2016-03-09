#pragma once

#include <zpatternprojectionplugin.h>

namespace Z3D
{

class ZBinaryPatternProjectionPlugin : public ZPatternProjectionPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "z3d.zstructuredlight.zstructuredlightsystemplugin" FILE "zbinarypatternprojectionplugin.json")
    Q_INTERFACES(Z3D::ZPatternProjectionPlugin)

public:
    ZBinaryPatternProjectionPlugin();

    // ZCorePlugin interface
public:
    virtual QString id() override;
    virtual QString name() override;
    virtual QString version() override;

    // ZPatternProjectionPlugin interface
public:
    virtual QList<ZPatternProjection *> getAll() override;
};

} // namespace Z3D
