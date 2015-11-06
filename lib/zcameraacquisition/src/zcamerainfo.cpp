#include "zcamerainfo.h"
#include "zcameraplugininterface.h"

namespace Z3D
{

ZCameraInfo::ZCameraInfo(ZCameraPluginInterface *plugin, QString name, QVariantMap extraData, QObject *parent)
    : QObject(parent)
    , m_plugin(plugin)
    , m_name(name)
    , m_extraData(extraData)
{

}

ZCameraInfo::~ZCameraInfo()
{

}

QString ZCameraInfo::name() const
{
    return m_name;
}

QString ZCameraInfo::pluginName() const
{
    return m_plugin->name();
}

QVariantMap ZCameraInfo::extraData() const
{
    return m_extraData;
}

ZCameraInterface::Ptr ZCameraInfo::getCamera() const
{
    return m_plugin->getCamera(m_extraData);
}

} // namespace Z3D
