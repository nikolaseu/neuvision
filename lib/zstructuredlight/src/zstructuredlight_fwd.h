#pragma once

#include <memory>
#include <QPointer>

namespace Z3D
{

class ZCameraAcquisitionManager;
struct ZDecodedPattern;
class ZPatternProjection;
class ZPatternProjectionPlugin;
struct ZProjectedPattern;
class ZStructuredLightSystem;
class ZStructuredLightSystemPlugin;

typedef std::shared_ptr<ZCameraAcquisitionManager> ZCameraAcquisitionManagerPtr;

typedef std::shared_ptr<ZDecodedPattern> ZDecodedPatternPtr;

typedef std::shared_ptr<ZPatternProjection> ZPatternProjectionPtr;
typedef QPointer<ZPatternProjection> ZPatternProjectionWeakPtr;

typedef std::shared_ptr<ZProjectedPattern> ZProjectedPatternPtr;

typedef std::shared_ptr<ZStructuredLightSystem> ZStructuredLightSystemPtr;
typedef QPointer<ZStructuredLightSystem> ZStructuredLightSystemWeakPtr;

} // namespace Z3D
