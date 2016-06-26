#include "zstereoslsplugin.h"

#include "zdualcamerastereosls.h"
#include "zsinglecamerastereosls.h"

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
    return QList<ZStructuredLightSystem *>()
            << new ZDualCameraStereoSLS()
            << new ZSingleCameraStereoSLS();
}

} // namespace Z3D
