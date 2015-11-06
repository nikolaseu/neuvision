#include "pinholecameracalibrationplugin.h"

#include "pinholecameracalibration.h"

namespace Z3D
{

QString PinholeCameraCalibrationPlugin::name()
{
    return QString("Pinhole");
}

QString PinholeCameraCalibrationPlugin::version()
{
    return QString("1.0");
}

CameraCalibrationInterface::Ptr PinholeCameraCalibrationPlugin::getCalibration(QVariantMap options)
{
    CameraCalibrationInterface::Ptr calibration(new PinholeCameraCalibration);

    if (calibration && options.contains("ConfigFile")) {
        QString configFileName = options.value("ConfigFile").toString();
        calibration->loadCalibration(configFileName);
    }

    return calibration;
}

} // namespace Z3D

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zpinholeplugin, Z3D::PinholeCameraCalibrationPlugin)
#endif // QT_VERSION < 0x050000

