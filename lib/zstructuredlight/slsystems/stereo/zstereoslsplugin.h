#ifndef Z3D_ZSTEREOSLSPLUGIN_H
#define Z3D_ZSTEREOSLSPLUGIN_H

#include "zstructuredlightsystemplugin.h"

namespace Z3D
{

class ZStereoSLSPlugin : public ZStructuredLightSystemPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "z3d.zstructuredlight.zstructuredlightsystemplugin" FILE "zstereoslsplugin.json")
    Q_INTERFACES(Z3D::ZStructuredLightSystemPlugin)

public:
    ZStereoSLSPlugin();

    // ZCorePlugin interface
public:
    virtual QString id() override;
    virtual QString name() override;
    virtual QString version() override;

    // ZStructuredLightSystemPlugin interface
public:
    virtual QList<ZStructuredLightSystem *> getAll() override;
};

} // namespace Z3D

#endif // Z3D_ZSTEREOSLSPLUGIN_H
