#include "zpinholecameracalibrationplugin.h"

#include "zpinholecameracalibration.h"

#include "zpinholecameracalibrator.h"
#include "zopencvcustomstereomulticameracalibrator.h"
#include "zopencvstereomulticameracalibrator.h"

namespace Z3D
{

QString ZPinholeCameraCalibrationPlugin::name()
{
    return QString(PINHOLE_CALIB_NAME);
}

QString ZPinholeCameraCalibrationPlugin::version()
{
    return QString(PINHOLE_CALIB_VERSION);
}

ZCameraCalibration::Ptr ZPinholeCameraCalibrationPlugin::getCalibration(QVariantMap options)
{
    ZCameraCalibration::Ptr calibration(new ZPinholeCameraCalibration);

    if (calibration && options.contains("ConfigFile")) {
        QString configFileName = options.value("ConfigFile").toString();
        calibration->loadFromFile(configFileName);
    }

    return calibration;
}

QList<ZCameraCalibrator *> ZPinholeCameraCalibrationPlugin::getCameraCalibrators()
{
    QList<ZCameraCalibrator *> list;

    list << new ZPinholeCameraCalibrator();

    return list;
}

QList<ZMultiCameraCalibrator *> ZPinholeCameraCalibrationPlugin::getMultiCameraCalibrators()
{
    QList<ZMultiCameraCalibrator *> list;

    list << new ZOpenCVCustomStereoMultiCameraCalibrator();
    list << new ZOpenCVStereoMultiCameraCalibrator();

    return list;
}

} // namespace Z3D

/*
#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(zpinholeplugin, Z3D::PinholeCameraCalibrationPlugin)
#endif // QT_VERSION < 0x050000
*/
