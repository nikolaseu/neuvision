#include "zstructuredlightsystemprovider.h"

#include "zpluginloader.h"
#include "zstructuredlightsystemplugin.h"

#include <QDebug>

namespace Z3D
{

QMap<QString, ZStructuredLightSystemPlugin *> ZStructuredLightSystemProvider::m_plugins;

void ZStructuredLightSystemProvider::loadPlugins()
{
    auto list = ZPluginLoader::plugins("structuredlight");

    for (auto pluginInstance : list) {
        ZStructuredLightSystemPlugin *plugin = qobject_cast<ZStructuredLightSystemPlugin *>(pluginInstance);
        if (plugin) {
            qDebug() << "structured light system plugin loaded. type:" << plugin->id()
                     << "version:" << plugin->version();
            m_plugins.insert(plugin->id(), plugin);
        } else {
            qWarning() << "invalid structured light system plugin:" << plugin;
        }
    }
}

void ZStructuredLightSystemProvider::unloadPlugins()
{

}

QList<ZStructuredLightSystem *> ZStructuredLightSystemProvider::getAll()
{
    QList<ZStructuredLightSystem *> list;
    for (auto plugin : m_plugins.values()) {
        list << plugin->getAll();
    }
    return list;
}

ZStructuredLightSystemProvider::ZStructuredLightSystemProvider()
{

}

} // namespace Z3D
