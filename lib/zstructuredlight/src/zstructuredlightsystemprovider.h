#ifndef Z3D_ZSTRUCTUREDLIGHTSYSTEMPROVIDER_H
#define Z3D_ZSTRUCTUREDLIGHTSYSTEMPROVIDER_H

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

    static QList<ZStructuredLightSystemPlugin*> m_list;
};

} // namespace Z3D

#endif // Z3D_ZSTRUCTUREDLIGHTSYSTEMPROVIDER_H
