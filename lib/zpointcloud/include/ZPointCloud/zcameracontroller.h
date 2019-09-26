#pragma once

#include "zpointcloud_global.h"

#include <QEntity>
#include <QMouseEvent>
#include <QVector3D>

class QSizeF;
namespace Qt3DInput {
class QMouseDevice;
class QKeyboardDevice;
class QMouseHandler;
class QKeyboardHandler;
class QMouseEvent;
class QKeyEvent;
};
namespace Qt3DRender {
class QCamera;
class QLayer;
class QPickTriangleEvent;
}
namespace Qt3DInput {
class QMouseEvent;
}
namespace Qt3DLogic {
class QFrameAction;
}

namespace Z3D
{

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZCameraController : public Qt3DCore::QEntity
{
    Q_OBJECT

    Q_PROPERTY(Qt3DRender::QCamera* camera READ camera WRITE setCamera NOTIFY cameraChanged)
    Q_PROPERTY(QRect viewport READ viewport WRITE setViewport NOTIFY viewportChanged)
//    Q_PROPERTY(Qt3DRender::QPickTriangleEvent* groundPick READ groundPick WRITE setGroundPick NOTIFY groundPickChanged)

public:
    explicit ZCameraController(Qt3DCore::QEntity* pParent = nullptr);
    Qt3DRender::QCamera* camera() const;
    void setCamera(Qt3DRender::QCamera* camera);
    Qt3DRender::QLayer* raycastLayer() const;
    void setRaycastLayer(Qt3DRender::QLayer* raycastLayer);
    QRect viewport() const;
    void setViewport(const QRect& viewport);
    Qt3DRender::QPickTriangleEvent* groundPick() const;
    void setGroundPick(Qt3DRender::QPickTriangleEvent* groundPick);

signals:
    void cameraChanged();
    void viewportChanged();
    void groundPickChanged();

private slots:
    void onMouseClicked(Qt3DInput::QMouseEvent* event);
    void onMousePressed(Qt3DInput::QMouseEvent* event);
    void onMouseRelease(Qt3DInput::QMouseEvent* event);
    void onMousePositionChanged(Qt3DInput::QMouseEvent* event);
    void onMouseWheel(Qt3DInput::QWheelEvent* event);
    void onKeyPressed(Qt3DInput::QKeyEvent* event);
    void onKeyRelease(Qt3DInput::QKeyEvent* event);
    void onLeftKeyPressed(Qt3DInput::QKeyEvent* event);
    void onUpKeyPressed(Qt3DInput::QKeyEvent* event);
    void onRightKeyPressed(Qt3DInput::QKeyEvent* event);
    void onDownKeyPressed(Qt3DInput::QKeyEvent* event);
    void onDeleteKeyPressed(Qt3DInput::QKeyEvent* event);
    void onTabKeyPressed(Qt3DInput::QKeyEvent* event);
//    void onHitsChanged(const Qt3DRender::QAbstractRayCaster::Hits &hits);
    void onFrameTriggered(float dt);

private:
    bool checkMouseDragging(const QPoint& p1, const QPoint& p2);
    void translate(const QPoint& p1, const QPoint& p2, float dt);
    void rotate(const QPoint& p1, const QPoint& p2, float dt);
    void reset();

    Qt3DInput::QMouseDevice* m_mouseDevice = nullptr;
    Qt3DInput::QKeyboardDevice* m_keyboardDevice = nullptr;
    Qt3DInput::QMouseHandler* m_mouseHandler = nullptr;
    Qt3DInput::QKeyboardHandler* m_keyboardHandler = nullptr;
    Qt3DLogic::QFrameAction* m_frameAction = nullptr;
    Qt3DRender::QCamera* m_camera = nullptr;
    Qt3DRender::QPickTriangleEvent* m_groundPick = nullptr;
    //Qt3DRender::QScreenRayCaster* _rayCaster = nullptr;

    QRect m_viewport;
    QPoint m_sceneCenter;

    Qt3DInput::QMouseEvent::Buttons _pressedMouseButton = Qt3DInput::QMouseEvent::Buttons::NoButton;
    std::vector<Qt::Key> _pressedKeys;
    std::vector<Qt::Key> _modifierKeys;
    bool _mouseDraggingActiv = false;
    QPoint _fromMousePixel;
    QPoint _toMousePixel;
    QVector3D _lastMouseWorldPosition;

    int _minZoom = 2;
    int _maxZoom = 10000;
    float _mouseZoomSpeed = 0.05f;
    float _mouseRotationSpeed = 0.0f; // Translation to the camera viewcenter results in 90 degree rotation
    int _mouseTranslationSpeed = 10;
    float _maxMouseRotationSpeed = 1.5f;
    int _mouseDraggingThresholdSquard = 75;
    int _keyTranslationSpeed = 500;
    int _keyRotationSpeed = 200;
};

} // namespace Z3D
