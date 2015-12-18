#include "zstereoslsplugin.h"

#include "zstereosls.h"

namespace Z3D {

ZStereoSLSPlugin::ZStereoSLSPlugin()
{

}

QString ZStereoSLSPlugin::id()
{
    return "ZStereoSLSPlugin";
}

QString ZStereoSLSPlugin::name()
{
    return "Stereo";
}

QString ZStereoSLSPlugin::version()
{
    return Z3D_VERSION_STR;
}

QList<ZStructuredLightSystem *> ZStereoSLSPlugin::getAll()
{
    return QList<ZStructuredLightSystem *>() << new ZStereoSLS();
}

} // namespace Z3D
