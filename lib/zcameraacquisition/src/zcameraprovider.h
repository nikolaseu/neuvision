#ifndef Z3D_CAMERAACQUISITION___ZCAMERAPROVIDER_H
#define Z3D_CAMERAACQUISITION___ZCAMERAPROVIDER_H

#include "zcameraacquisition_global.h"
#include "zcamerainterface.h"
#include "zcameralistmodel.h"

#include <QMap>
#include <QSettings>
#include <QVariantMap>

namespace Z3D
{

class ZCameraPluginInterface;

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraProvider
{

public:
    static void loadPlugins(QString folder = QString());
    static void unloadPlugins();

    static ZCameraInterface::Ptr getCamera(QString pluginName, QVariantMap options);

    static ZCameraInterface::Ptr getCamera(QSettings *settings);

    static QList<Z3D::ZCameraInterface::Ptr> loadCameras(QString folder = QString());

    static Z3D::ZCameraListModel *model();

    static QList<ZCameraPluginInterface *> availablePlugins();

private:
    explicit ZCameraProvider() {}

    static QMap< QString, ZCameraPluginInterface *> m_plugins;

    static Z3D::ZCameraListModel m_model;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION___ZCAMERAPROVIDER_H
