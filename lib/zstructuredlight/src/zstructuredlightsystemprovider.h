#pragma once

#include "zstructuredlight_global.h"

namespace Z3D
{

class ZStructuredLightSystem;
class ZStructuredLightSystemPlugin;

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZStructuredLightSystemProvider
{
public:
    static void loadPlugins();
    static void unloadPlugins();

    static QList<ZStructuredLightSystem*> getAll();

private:
    explicit ZStructuredLightSystemProvider();

    static QMap< QString, ZStructuredLightSystemPlugin *> m_plugins;
};

} // namespace Z3D
