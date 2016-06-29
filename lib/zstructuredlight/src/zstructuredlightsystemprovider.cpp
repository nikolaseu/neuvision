#include "zstructuredlightsystemprovider.h"

#include "zpluginloader.h"
#include "zstructuredlightsystemplugin.h"

#include <QDebug>
#include <QSettings>

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

QList<QString> ZStructuredLightSystemProvider::getAll()
{
    QList<QString> list;
    for (auto plugin : m_plugins.values()) {
        list << plugin->name();
    }
    return list;
}

ZStructuredLightSystem::Ptr ZStructuredLightSystemProvider::get(QSettings *settings)
{
    ZStructuredLightSystem::Ptr structuredLightSystem;

    settings->beginGroup("StructuredLightSystem");
    {
        const QString pluginId = settings->value("Type").toString();
        if (m_plugins.contains(pluginId)) {
            const auto plugin = m_plugins.value(pluginId);
            structuredLightSystem = plugin->get(settings);
        }
    }
    settings->endGroup();

    return structuredLightSystem;
}

ZStructuredLightSystemProvider::ZStructuredLightSystemProvider()
{

}

} // namespace Z3D
