#include "zbinarypatternprojectionplugin.h"
#include "zbinarypatternprojection.h"

namespace Z3D
{

ZBinaryPatternProjectionPlugin::ZBinaryPatternProjectionPlugin()
{

}

QString ZBinaryPatternProjectionPlugin::id()
{
    return "ZBinaryPatternProjectionPlugin";
}

QString ZBinaryPatternProjectionPlugin::name()
{
    return "Binary";
}

QString ZBinaryPatternProjectionPlugin::version()
{
    return Z3D_VERSION_STR;
}

QList<ZPatternProjection *> ZBinaryPatternProjectionPlugin::getAll()
{
    return QList<ZPatternProjection *>() << new ZBinaryPatternProjection();
}

} // namespace Z3D
