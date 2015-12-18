#include "zpatternprojectionprovider.h"

#include "zpluginloader.h"
#include "zpatternprojectionplugin.h"

#include <QDebug>

namespace Z3D
{

QList<ZPatternProjectionPlugin*> ZPatternProjectionProvider::m_list;

void ZPatternProjectionProvider::loadPlugins()
{
    auto list = ZPluginLoader::plugins("structuredlightpatterns");

    for (auto pluginInstance : list) {
        auto *plugin = qobject_cast<ZPatternProjectionPlugin *>(pluginInstance);
        if (plugin) {
            qDebug() << "pattern projection plugin loaded. type:" << plugin->id()
                     << "version:" << plugin->version();
            m_list << plugin;
        } else {
            qWarning() << "invalid pattern projection plugin:" << plugin;
        }
    }
}

void ZPatternProjectionProvider::unloadPlugins()
{

}

QList<ZPatternProjection *> ZPatternProjectionProvider::getAll()
{
    QList<ZPatternProjection *> list;
    for (auto plugin : m_list) {
        list << plugin->getAll();
    }
    return list;
}

ZPatternProjectionProvider::ZPatternProjectionProvider()
{
}

} // namespace Z3D
