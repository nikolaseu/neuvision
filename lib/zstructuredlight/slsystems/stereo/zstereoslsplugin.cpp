#include "zstereoslsplugin.h"

#include "zdualcamerastereosls.h"
#include "zsinglecamerastereosls.h"

namespace Z3D {

ZStereoSLSPlugin::ZStereoSLSPlugin()
{

}

QString ZStereoSLSPlugin::id()
{
    return "ZStereoSLS";
}

QString ZStereoSLSPlugin::name()
{
    return "Stereo";
}

QString ZStereoSLSPlugin::version()
{
    return Z3D_VERSION_STR;
}

QList<QString> ZStereoSLSPlugin::getAll()
{
    return QList<QString>()
            << "DualCamera"
            << "Projector+Camera";
}

ZStructuredLightSystem::Ptr ZStereoSLSPlugin::get(QSettings *settings)
{
    ZStructuredLightSystem::Ptr sls;

    const QString mode = settings->value("Mode").toString();
    if (mode == "DualCamera") {
        sls = ZStructuredLightSystem::Ptr(new ZDualCameraStereoSLS());
    } else if (mode == "Projector+Camera") {
        sls = ZStructuredLightSystem::Ptr(new ZSingleCameraStereoSLS());
    }

    if (sls) {
        sls->init(settings);
    }

    return sls;
}

} // namespace Z3D
