#include "ZPointCloud/zcameracontroller.h"

#include <QSizeF>
#include <Qt3DInput/QKeyEvent>
#include <Qt3DInput/QKeyboardDevice>
#include <Qt3DInput/QKeyboardHandler>
#include <Qt3DInput/QMouseDevice>
#include <Qt3DInput/QMouseEvent>
#include <Qt3DInput/QMouseHandler>
#include <Qt3DLogic/QFrameAction>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QLayer>
#include <Qt3DRender/QPickTriangleEvent>
#include <iostream>

namespace Z3D
{

ZCameraController::ZCameraController(Qt3DCore::QEntity* pParent)
    : Qt3DCore::QEntity(pParent)
    , m_mouseDevice(new Qt3DInput::QMouseDevice(this))
    , m_keyboardDevice(new Qt3DInput::QKeyboardDevice(this))
    , m_mouseHandler(new Qt3DInput::QMouseHandler(this))
    , m_keyboardHandler(new Qt3DInput::QKeyboardHandler(this))
    ,
    //_rayCaster(new Qt3DRender::QScreenRayCaster(this)),
    m_frameAction(new Qt3DLogic::QFrameAction(this))
//    , m_ground({ 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 10000)
{
    m_mouseHandler->setSourceDevice(m_mouseDevice);
    if (!connect(m_mouseHandler, &Qt3DInput::QMouseHandler::clicked,
                 this, &ZCameraController::onMouseClicked))
        assert(0);
    if (!connect(m_mouseHandler, &Qt3DInput::QMouseHandler::pressed,
                 this, &ZCameraController::onMousePressed))
        assert(0);
    if (!connect(m_mouseHandler, &Qt3DInput::QMouseHandler::released,
                 this, &ZCameraController::onMouseRelease))
        assert(0);
    if (!connect(m_mouseHandler, &Qt3DInput::QMouseHandler::positionChanged,
                 this, &ZCameraController::onMousePositionChanged))
        assert(0);
    if (!connect(m_mouseHandler, &Qt3DInput::QMouseHandler::wheel,
                 this, &ZCameraController::onMouseWheel))
        assert(0);

    m_keyboardHandler->setSourceDevice(m_keyboardDevice);
    m_keyboardHandler->setFocus(true);
    if (!connect(m_keyboardHandler, &Qt3DInput::QKeyboardHandler::pressed,
                 this, &ZCameraController::onKeyPressed))
        assert(0);
    if (!connect(m_keyboardHandler, &Qt3DInput::QKeyboardHandler::released,
                 this, &ZCameraController::onKeyRelease))
        assert(0);
    if (!connect(m_keyboardHandler, &Qt3DInput::QKeyboardHandler::leftPressed,
                 this, &ZCameraController::onLeftKeyPressed))
        assert(0);
    if (!connect(m_keyboardHandler, &Qt3DInput::QKeyboardHandler::upPressed,
                 this, &ZCameraController::onUpKeyPressed))
        assert(0);
    if (!connect(m_keyboardHandler, &Qt3DInput::QKeyboardHandler::rightPressed,
                 this, &ZCameraController::onRightKeyPressed))
        assert(0);
    if (!connect(m_keyboardHandler, &Qt3DInput::QKeyboardHandler::downPressed,
                 this, &ZCameraController::onDownKeyPressed))
        assert(0);
    if (!connect(m_keyboardHandler, &Qt3DInput::QKeyboardHandler::deletePressed,
                 this, &ZCameraController::onDeleteKeyPressed))
        assert(0);
    if (!connect(m_keyboardHandler, &Qt3DInput::QKeyboardHandler::tabPressed,
                 this, &ZCameraController::onTabKeyPressed))
        assert(0);

//    //_rayCaster->setFilterMode(Qt3DRender::QAbstractRayCaster::AcceptAllMatchingLayers);
//    m_rayCaster.setEntity(&m_ground);

//    //if (!connect(_rayCaster, &Qt3DRender::QScreenRayCaster::hitsChanged, this, &MapCameraController::onHitsChanged)) assert(0);
//    if (!connect(&m_rayCaster, &CameraRayCaster::hitsChanged, this, &MyCameraController::onHitsChanged))
//        assert(0);
    if (!connect(m_frameAction, &Qt3DLogic::QFrameAction::triggered, this, &ZCameraController::onFrameTriggered))
        assert(0);

    //addComponent(_rayCaster);
    addComponent(m_frameAction);
}

Qt3DRender::QCamera* ZCameraController::camera() const
{
    return m_camera;
}

void ZCameraController::setCamera(Qt3DRender::QCamera* camera)
{
    if (m_camera == camera) {
        return;
    }

    if (camera && !camera->parent()) {
        camera->setParent(this);
    }

//    m_rayCaster.setCamera(camera);
    m_camera = camera;

//    connect(m_camera, &Qt3DRender::QCamera::positionChanged, [this](const QVector3D& position) {
//        m_ground.setOrigin({ position.x(), position.y(), 0.0f });
//    });

    emit cameraChanged();
}

QRect ZCameraController::viewport() const
{
    return m_viewport;
}

void ZCameraController::setViewport(const QRect& viewport)
{
    if (m_viewport == viewport) {
        return;
    }

    m_sceneCenter = viewport.center() + QPoint(1, 1);
//    m_rayCaster.setViewport(viewport);
    m_viewport = viewport;

    emit viewportChanged();
}

Qt3DRender::QPickTriangleEvent* ZCameraController::groundPick() const
{
    return m_groundPick;
}

void ZCameraController::setGroundPick(Qt3DRender::QPickTriangleEvent* groundPick)
{
    if (m_groundPick == groundPick)
        return;

    m_groundPick = groundPick;
    emit groundPickChanged();
}

bool ZCameraController::checkMouseDragging(const QPoint& p1, const QPoint& p2)
{
    if (_mouseDraggingActiv)
        return true;

    auto dx = p2.x() - p1.x();
    auto dy = p2.y() - p1.y();
    if (dx * dx > _mouseDraggingThresholdSquard || dy * dy > _mouseDraggingThresholdSquard) {
        _mouseDraggingActiv = true;
    }

    return _mouseDraggingActiv;
}

void ZCameraController::translate(const QPoint& p1, const QPoint& p2, float factor)
{
    QVector3D translation { p2 - p1 };
    m_camera->translateWorld(translation * factor, Qt3DRender::QCamera::TranslateViewCenter);
}

void ZCameraController::rotate(const QPoint& p1, const QPoint& p2, float factor)
{
    auto pan = (p1.x() - p2.x()) * factor;
    auto tilt = (p2.y() - p1.y()) * factor;

    bool doTilt = true;
    if (m_camera->upVector().z() <= 0.0f && tilt > 0.0f)
        doTilt = false;
    else if (m_camera->upVector().z() >= 0.999f && tilt < 0.001f)
        doTilt = false;

    m_camera->panAboutViewCenter(pan, { 0.0, 0.0, 1.0 });

    if (doTilt)
        m_camera->tiltAboutViewCenter(tilt);
}

void ZCameraController::reset()
{
    _pressedMouseButton = Qt3DInput::QMouseEvent::Buttons::NoButton;
    _fromMousePixel = {};
    _toMousePixel = {};
    _lastMouseWorldPosition = {};
    _mouseDraggingActiv = false;
}

void ZCameraController::onMouseClicked(Qt3DInput::QMouseEvent* event)
{
    qDebug("Clicked");
}

void ZCameraController::onMousePressed(Qt3DInput::QMouseEvent* event)
{
    reset();

    _pressedMouseButton = event->button();
    _fromMousePixel = { event->x(), event->y() };
    _mouseRotationSpeed = 90.0f / (QVector2D(m_sceneCenter) - QVector2D(_fromMousePixel)).length();
    _mouseRotationSpeed = _mouseRotationSpeed > _maxMouseRotationSpeed ? _maxMouseRotationSpeed : _mouseRotationSpeed;

//    m_rayCaster.trigger(_fromMousePixel);
}

void ZCameraController::onMouseRelease(Qt3DInput::QMouseEvent* event)
{
    reset();
}

void ZCameraController::onMousePositionChanged(Qt3DInput::QMouseEvent* event)
{
    if (_pressedMouseButton != Qt3DInput::QMouseEvent::Buttons::NoButton) {
        _toMousePixel = { event->x(), event->y() };
    }
}

void ZCameraController::onMouseWheel(Qt3DInput::QWheelEvent* event)
{
    float distance = (m_camera->viewCenter() - m_camera->position()).length();
    float sign = event->angleDelta().y() >= 0 ? 1 : -1;

    if (distance < _minZoom && sign > 0) {
        return;
    }
    if (distance > _maxZoom && sign < 0) {
        return;
    }

    auto zoom = sign * distance * _mouseZoomSpeed;
    auto newPosition = m_camera->position() + m_camera->viewVector().normalized() * zoom;
    m_camera->setPosition(newPosition);
}

void ZCameraController::onKeyPressed(Qt3DInput::QKeyEvent* event)
{
    if (event->key() == Qt::Key_Control) {
        _modifierKeys.emplace_back(static_cast<Qt::Key>(event->key()));
        event->setAccepted(true);
    }
}

void ZCameraController::onKeyRelease(Qt3DInput::QKeyEvent* event)
{
    if (event->key() == Qt::Key_Control) {
        //qDebug() << "Control Key released";
        _modifierKeys.erase(std::remove(begin(_modifierKeys), end(_modifierKeys), event->key()), end(_modifierKeys));
        event->setAccepted(true);
    }

    if (event->key() == Qt::Key_Left || event->key() == Qt::Key_Up || event->key() == Qt::Key_Right || event->key() == Qt::Key_Down) {
        //qDebug() << "Left Key released";
        _pressedKeys.erase(std::remove(begin(_pressedKeys), end(_pressedKeys), event->key()), end(_pressedKeys));
        event->setAccepted(true);
    }
}

void ZCameraController::onLeftKeyPressed(Qt3DInput::QKeyEvent* event)
{
    Qt::Key key = static_cast<Qt::Key>(event->key());
    _pressedKeys.emplace_back(key);
    event->setAccepted(true);
}

void ZCameraController::onUpKeyPressed(Qt3DInput::QKeyEvent* event)
{
    Qt::Key key = static_cast<Qt::Key>(event->key());
    _pressedKeys.emplace_back(key);
    event->setAccepted(true);
}

void ZCameraController::onRightKeyPressed(Qt3DInput::QKeyEvent* event)
{
    Qt::Key key = static_cast<Qt::Key>(event->key());
    _pressedKeys.emplace_back(key);
    event->setAccepted(true);
}

void ZCameraController::onDownKeyPressed(Qt3DInput::QKeyEvent* event)
{
    Qt::Key key = static_cast<Qt::Key>(event->key());
    _pressedKeys.emplace_back(key);
    event->setAccepted(true);
}

void ZCameraController::onDeleteKeyPressed(Qt3DInput::QKeyEvent* event)
{
    Qt::Key key = static_cast<Qt::Key>(event->key());
    _pressedKeys.emplace_back(key);
}

void ZCameraController::onTabKeyPressed(Qt3DInput::QKeyEvent* event)
{
    Qt::Key key = static_cast<Qt::Key>(event->key());
    _pressedKeys.emplace_back(key);
}

//void MyCameraController::onHitsChanged(const Qt3DRender::QAbstractRayCaster::Hits& hits)
//{
//    if (hits.isEmpty()) {
//        qDebug() << "No intersection with ground plane";
//        return;
//    }

//    if (hits.length() > 1) {
//        qDebug() << "More than one intersection with ground plane";
//        //for (auto hit : hits)
//        //    qDebug() << "hit " << hit.worldIntersection();
//        //return;
//    }

//    if (_pressedMouseButton != Qt3DInput::QMouseEvent::Buttons::NoButton) {
//        if (!_mouseDraggingActiv)
//            return;

//        if (_lastMouseWorldPosition.isNull()) {
//            _lastMouseWorldPosition = hits[0].worldIntersection();
//            return;
//        }

//        if (_pressedMouseButton == Qt3DInput::QMouseEvent::Buttons::LeftButton) {
//            auto translation = _lastMouseWorldPosition - hits[0].worldIntersection();
//            translation.setZ(0);
//            m_camera->translateWorld(translation, Qt3DRender::QCamera::TranslateViewCenter);
//            _lastMouseWorldPosition = hits[0].worldIntersection() + translation;
//        }
//    }

//    if (!_pressedKeys.empty()) {
//        auto translation = m_camera->viewCenter() - hits[0].worldIntersection();
//        translation.setZ(0);
//        m_camera->translateWorld(translation, Qt3DRender::QCamera::TranslateViewCenter);
//    }
//}

void ZCameraController::onFrameTriggered(float dt)
{
    if (_pressedMouseButton != Qt3DInput::QMouseEvent::Buttons::NoButton) {
        if (_toMousePixel.isNull())
            return;

        if (!checkMouseDragging(_fromMousePixel, _toMousePixel))
            return;

        if (_pressedMouseButton == Qt3DInput::QMouseEvent::Buttons::LeftButton) {
//            //translate(_fromMousePixel, _toMousePixel, _mouseTranslationSpeed * dt);
//            //_fromMousePixel = _toMousePixel;
//            m_rayCaster.trigger(_toMousePixel);
        } else if (_pressedMouseButton == Qt3DInput::QMouseEvent::Buttons::RightButton) {
            rotate(_fromMousePixel, _toMousePixel, _mouseRotationSpeed);
            _fromMousePixel = _toMousePixel;
        }
    } else if (!_pressedKeys.empty()) {
        QPoint fromPixel = m_sceneCenter;
        QPoint toPixel = m_sceneCenter;
        auto controlPressed = std::find(begin(_modifierKeys), end(_modifierKeys), Qt::Key_Control) != end(_modifierKeys);

        int step = 1;
        if (controlPressed) {
            step = qRound(_keyRotationSpeed * dt);
        } else {
            step = qRound(_keyTranslationSpeed * dt);
        }

        for (auto key : _pressedKeys) {
            if (key == Qt::Key_Left)
                toPixel.rx() -= step;
            if (key == Qt::Key_Right)
                toPixel.rx() += step;
            if (key == Qt::Key_Up)
                toPixel.ry() -= step;
            if (key == Qt::Key_Down)
                toPixel.ry() += step;
        }

        if (controlPressed) {
            rotate(fromPixel, toPixel, 1);
        } else {
//            m_rayCaster.trigger(toPixel);
//            //translate(fromPixel, toPixel, _keyTranslationSpeed * dt);
        }
    }
}

} // namespace Z3D
