#ifndef Z3D_CAMERACALIBRATION___ZCAMERACALIBRATIONPROVIDER_H
#define Z3D_CAMERACALIBRATION___ZCAMERACALIBRATIONPROVIDER_H

#include "zcameracalibration_global.h"
#include "zcameracalibration.h"
#include "zcameracalibrator.h"
#include "zmulticameracalibrator.h"

#include <QMap>
#include <QSettings>
#include <QVariantMap>

namespace Z3D
{

class ZCameraCalibrationPluginInterface;

class Z3D_CAMERACALIBRATION_SHARED_EXPORT ZCameraCalibrationProvider
{

public:
    static void loadPlugins(QString folder = QString());
    static void unloadPlugins();

    static ZCameraCalibration::Ptr getCalibration(QString pluginName, QVariantMap options);

    static ZCameraCalibration::Ptr getCalibration(QSettings *settings);

    static QList<ZCameraCalibrator *> getCameraCalibrators();

    static QList<ZMultiCameraCalibrator *> getMultiCameraCalibrators();

private:
    explicit ZCameraCalibrationProvider() {}

    static QMap< QString, ZCameraCalibrationPluginInterface *> m_plugins;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATION___ZCAMERACALIBRATIONPROVIDER_H
