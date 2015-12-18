#ifndef Z3D_STRUCTUREDLIGHT___ZPATTERNPROJECTIONPROVIDER_H
#define Z3D_STRUCTUREDLIGHT___ZPATTERNPROJECTIONPROVIDER_H

#include "zstructuredlight_global.h"

namespace Z3D
{

class ZPatternProjection;
class ZPatternProjectionPlugin;

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZPatternProjectionProvider
{

public:
    static void loadPlugins();
    static void unloadPlugins();

    static QList<ZPatternProjection*> getAll();

private:
    explicit ZPatternProjectionProvider();

    static QList<ZPatternProjectionPlugin*> m_list;
};

} // namespace Z3D

#endif // Z3D_STRUCTUREDLIGHT___ZPATTERNPROJECTIONPROVIDER_H
