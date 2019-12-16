#pragma once

#include <memory>
#include <QPointer>

namespace Z3D
{

class ZCalibrationImage;
class ZCalibrationImageModel;
class ZCalibrationImageViewer;
class ZCalibrationPatternFinder;
class ZCalibrationPatternFinderPluginInterface;
class ZCameraCalibratorWorker;
class ZMultiCalibrationImage;
class ZMultiCalibrationImageModel;
class ZMultiCameraCalibratorWorker;

typedef std::shared_ptr<ZCalibrationImage> ZCalibrationImagePtr;
typedef QPointer<ZCalibrationImage> ZCalibrationImageWeakPtr;

typedef QPointer<ZCalibrationImageModel> ZCalibrationImageModelWeakPtr;

typedef std::shared_ptr<ZCalibrationPatternFinder> ZCalibrationPatternFinderPtr;
typedef QPointer<ZCalibrationPatternFinder> ZCalibrationPatternFinderWeakPtr;

typedef std::shared_ptr<ZMultiCalibrationImage> ZMultiCalibrationImagePtr;

typedef QPointer<ZMultiCalibrationImageModel> ZMultiCalibrationImageModelWeakPtr;

} // namespace Z3D
