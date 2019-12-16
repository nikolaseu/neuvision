#pragma once

#include <memory>
#include <QPointer>

namespace Z3D
{

class ZCameraFramesRecorder;
class ZCameraInfo;
class ZCameraInterface;
class ZCameraListModel;
class ZCameraPreviewer;
class ZCameraProvider;
class ZCameraSelectorDialog;
class ZCameraSelectorWidget;
class ZCameraSettingsWidget;
class ZImageGrayscale;
class ZImageViewer;

typedef std::shared_ptr<ZCameraInterface> ZCameraPtr;
typedef QPointer<ZCameraInterface> ZCameraWeakPtr;
typedef std::vector<ZCameraPtr> ZCameraList;

typedef std::shared_ptr<ZImageGrayscale> ZCameraImagePtr;

} // namespace Z3D
