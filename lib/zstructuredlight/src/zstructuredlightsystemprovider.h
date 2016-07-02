#pragma once

#include "zstructuredlight_global.h"

#include "zstructuredlightsystem.h"

namespace Z3D
{

class ZStructuredLightSystemPlugin;

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZStructuredLightSystemProvider
{
public:
    static void loadPlugins();
    static void unloadPlugins();

    static QList<QString> getAll();

    static Z3D::ZStructuredLightSystem::Ptr get(QSettings *settings);

private:
    explicit ZStructuredLightSystemProvider();

    static QMap< QString, ZStructuredLightSystemPlugin *> m_plugins;
};

} // namespace Z3D
