#pragma once

#include "zcameraacquisition_global.h"
#include "zcamerainterface.h"

#include <QObject>
#include <QVariantMap>

namespace Z3D
{

/// FW declaration to avoid cyclic includes
class ZCameraPluginInterface;

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraInfo : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString name          READ name       CONSTANT)
    Q_PROPERTY(QString pluginName    READ pluginName CONSTANT)
    Q_PROPERTY(QVariantMap extraData READ extraData  CONSTANT)

public:
    ZCameraInfo(ZCameraPluginInterface *plugin, QString name, QVariantMap extraData, QObject *parent = 0);
    ~ZCameraInfo();

    QString name() const;
    QString pluginName() const;
    QVariantMap extraData() const;

    ZCameraInterface::Ptr getCamera() const;

private:
    ZCameraPluginInterface* m_plugin;

    QString m_name;
    QVariantMap m_extraData;
};

} // namespace Z3D
