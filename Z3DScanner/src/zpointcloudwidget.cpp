//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "zpointcloudwidget.h"
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>

ZPointCloudWidget::ZPointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent),
      m_perspectiveProjection(false),
      m_xRot(0),
      m_yRot(0),
      m_zRot(0),
      m_scale(1.f),
      m_pointSize(1),
      m_pointCloud(nullptr),
      m_program(nullptr)
{
    m_core = QCoreApplication::arguments().contains(QStringLiteral("--coreprofile"));

    setFocusPolicy(Qt::StrongFocus);
}

ZPointCloudWidget::~ZPointCloudWidget()
{
    cleanup();
}

QSize ZPointCloudWidget::minimumSizeHint() const
{
    return QSize(360, 240);
}

QSize ZPointCloudWidget::sizeHint() const
{
    return QSize(400, 400);
}

void ZPointCloudWidget::setPointCloudData(ZPointCloudData::Ptr pointCloudData)
{
    m_pointCloud = pointCloudData;

    setupVertexAttribs();
    update();
}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

void ZPointCloudWidget::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_xRot) {
        m_xRot = angle;
        emit xRotationChanged(angle);
        update();
    }
}

void ZPointCloudWidget::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_yRot) {
        m_yRot = angle;
        emit yRotationChanged(angle);
        update();
    }
}

void ZPointCloudWidget::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_zRot) {
        m_zRot = angle;
        emit zRotationChanged(angle);
        update();
    }
}

void ZPointCloudWidget::setScale(float scale)
{
    if (scale != m_scale && scale > 0.1f) {
        m_scale = scale;
        emit scaleChanged(scale);
        update();
    }
}

void ZPointCloudWidget::setPointSize(int pointSize)
{
    if (pointSize != m_pointSize && pointSize > 0 && pointSize < 10) {
        m_pointSize = pointSize;
        emit pointSizeChanged(pointSize);
        update();
    }
}

void ZPointCloudWidget::cleanup()
{
    makeCurrent();
    if (m_pointCloud)
        m_pointCloudVbo.destroy();
    delete m_program;
    m_program = 0;
    doneCurrent();
}

static const char *vertexShaderSourceCore =
    "#version 150 core\n"
    "in vec4 vertex;\n"
    "in vec3 color;\n"
    "out vec3 vert;\n"
    "out vec3 vertColor;\n"
    "uniform mat4 projMatrix;\n"
    "uniform mat4 mvMatrix;\n"
    "uniform mat3 normalMatrix;\n"
    "void main() {\n"
    "   vert = vertex.xyz;\n"
    "   vertColor = color;\n"
    "   gl_Position = projMatrix * mvMatrix * vertex;\n"
    "}\n";

static const char *fragmentShaderSourceCore =
    "#version 150 core\n"
    "in highp vec3 vert;\n"
    "in highp vec3 vertColor;\n"
    "out highp vec4 fragColor;\n"
    //"uniform highp vec3 lightPos;\n"
    "void main() {\n"
    //"   highp vec3 L = normalize(lightPos - vert);\n"
    //"   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
    //"   highp vec3 color = vec3(0.39, 1.0, 0.0);\n"
    //"   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
    "   fragColor = vec4(vertColor, 1.0);\n"
    "}\n";

/*
static const char *vertexShaderSourceCore =
    "#version 150\n"
    "in vec4 vertex;\n"
    "in vec3 normal;\n"
    "out vec3 vert;\n"
    "out vec3 vertNormal;\n"
    "uniform mat4 projMatrix;\n"
    "uniform mat4 mvMatrix;\n"
    "uniform mat3 normalMatrix;\n"
    "void main() {\n"
    "   vert = vertex.xyz;\n"
    "   vertNormal = normalMatrix * normal;\n"
    "   gl_Position = projMatrix * mvMatrix * vertex;\n"
    "}\n";

static const char *fragmentShaderSourceCore =
    "#version 150\n"
    "in highp vec3 vert;\n"
    "in highp vec3 vertNormal;\n"
    "out highp vec4 fragColor;\n"
    "uniform highp vec3 lightPos;\n"
    "void main() {\n"
    "   highp vec3 L = normalize(lightPos - vert);\n"
    "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
    "   highp vec3 color = vec3(0.39, 1.0, 0.0);\n"
    "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
    "   fragColor = vec4(col, 1.0);\n"
    "}\n";

static const char *vertexShaderSource =
    "attribute vec4 vertex;\n"
    "attribute vec3 normal;\n"
    "varying vec3 vert;\n"
    "varying vec3 vertNormal;\n"
    "uniform mat4 projMatrix;\n"
    "uniform mat4 mvMatrix;\n"
    "uniform mat3 normalMatrix;\n"
    "void main() {\n"
    "   vert = vertex.xyz;\n"
    "   vertNormal = normalMatrix * normal;\n"
    "   gl_Position = projMatrix * mvMatrix * vertex;\n"
    "}\n";

static const char *fragmentShaderSource =
    "varying highp vec3 vert;\n"
    "varying highp vec3 vertNormal;\n"
    "uniform highp vec3 lightPos;\n"
    "void main() {\n"
    "   highp vec3 L = normalize(lightPos - vert);\n"
    "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
    "   highp vec3 color = vec3(0.39, 1.0, 0.0);\n"
    "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
    "   gl_FragColor = vec4(col, 1.0);\n"
    "}\n";
*/

static const char *vertexShaderSource =
    "attribute vec4 vertex;\n"
    "attribute vec3 color;\n"
    "varying vec3 vert;\n"
    "varying vec3 vertColor;\n"
    "uniform mat4 projMatrix;\n"
    "uniform mat4 mvMatrix;\n"
    "uniform mat3 normalMatrix;\n"
    "void main() {\n"
    "   vert = vertex.xyz;\n"
    "   vertColor = color;\n"
    "   gl_Position = projMatrix * mvMatrix * vertex;\n"
    "}\n";

static const char *fragmentShaderSource =
    "varying highp vec3 vert;\n"
    "varying highp vec3 vertColor;\n"
    //"uniform highp vec3 lightPos;\n"
    "void main() {\n"
    //"   highp vec3 L = normalize(lightPos - vert);\n"
    //"   highp vec3 color = vec3(intensity, intensity, intensity);\n"
    //"   gl_FragColor = vec4(color, 1.0);\n"
    "   gl_FragColor = vec4(vertColor, 1.0);\n"
    "}\n";

void ZPointCloudWidget::initializeGL()
{
    // In this example the widget's corresponding top-level window can change
    // several times during the widget's lifetime. Whenever this happens, the
    // QOpenGLWidget's associated context is destroyed and a new one is created.
    // Therefore we have to be prepared to clean up the resources on the
    // aboutToBeDestroyed() signal, instead of the destructor. The emission of
    // the signal will be followed by an invocation of initializeGL() where we
    // can recreate all resources.
    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &ZPointCloudWidget::cleanup);

    initializeOpenGLFunctions();
    glClearColor(0, 0, 0, 1);

    m_program = new QOpenGLShaderProgram;
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, m_core ? vertexShaderSourceCore : vertexShaderSource);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, m_core ? fragmentShaderSourceCore : fragmentShaderSource);
    m_program->bindAttributeLocation("vertex", 0);
    m_program->bindAttributeLocation("color", 1);
    m_program->link();

    m_program->bind();
    m_projMatrixLoc = m_program->uniformLocation("projMatrix");
    m_mvMatrixLoc = m_program->uniformLocation("mvMatrix");
    m_normalMatrixLoc = m_program->uniformLocation("normalMatrix");
    m_lightPosLoc = m_program->uniformLocation("lightPos");

    // Create a vertex array object. In OpenGL ES 2.0 and OpenGL 2.x
    // implementations this is optional and support may not be present
    // at all. Nonetheless the below code works in all cases and makes
    // sure there is a VAO when one is needed.
    m_vao.create();
    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);

    // Our camera never changes in this example.
    m_camera.setToIdentity();
    m_camera.translate(0, 0, -1);

    // Light position is fixed.
    m_program->setUniformValue(m_lightPosLoc, QVector3D(0, 0, 700));

    m_program->release();
}

void ZPointCloudWidget::setupVertexAttribs()
{
    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);

    // Setup our vertex buffer object.
    if (m_pointCloudVbo.create() && m_pointCloudVbo.bind()) {
        m_pointCloudVbo.allocate(m_pointCloud->constData(), m_pointCloud->count() * sizeof(GLfloat));

        // Store the vertex attribute bindings for the program.
        m_pointCloudVbo.bind();
        QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
        f->glEnableVertexAttribArray(0);
        f->glEnableVertexAttribArray(1);
        f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), 0);
        f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));
        m_pointCloudVbo.release();
    } else {
        qWarning() << "Unable to create/bind VBO";
    }
}

void ZPointCloudWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    m_world.setToIdentity();
    m_world.rotate(180.0f - (m_xRot / 16.0f), 1, 0, 0);
    m_world.rotate(m_yRot / 16.0f, 0, 1, 0);
    m_world.rotate(m_zRot / 16.0f, 0, 0, 1);

    m_camera.setToIdentity();
    m_camera.translate(0, 0, -1000);
    m_camera.scale(m_scale);

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
    m_program->bind();
    m_program->setUniformValue(m_projMatrixLoc, m_proj);
    m_program->setUniformValue(m_mvMatrixLoc, m_camera * m_world);
    QMatrix3x3 normalMatrix = m_world.normalMatrix();
    m_program->setUniformValue(m_normalMatrixLoc, normalMatrix);

    if (m_pointCloud) {
#if !defined(Q_OS_ANDROID)
        glPointSize(devicePixelRatio() * m_pointSize);
#endif
        glDrawArrays(GL_POINTS, 0, m_pointCloud->vertexCount());
    }

    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        qWarning() << "OpenGL error:" << error;
    }

    m_program->release();
}

void ZPointCloudWidget::resizeGL(int w, int h)
{
    m_proj.setToIdentity();
    GLfloat aspectRatio = GLfloat(w) / h;
    if (m_perspectiveProjection)
        m_proj.perspective(45.0f, aspectRatio, 0.01f, 10000.0f);
    else
        m_proj.ortho(-aspectRatio, aspectRatio, -1.f, 1.f, 0.01f, 10000.0f);
}

void ZPointCloudWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->pos();
}

void ZPointCloudWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - m_lastPos.x();
    int dy = event->y() - m_lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(m_xRot + 8 * dy);
        setYRotation(m_yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(m_xRot + 8 * dy);
        setZRotation(m_zRot + 8 * dx);
    }
    m_lastPos = event->pos();
}

void ZPointCloudWidget::wheelEvent(QWheelEvent *event)
{
    if (event->modifiers() & Qt::ControlModifier) {
        setScale(m_scale + 0.01f * event->delta());
        return;
    } else if (event->modifiers() & Qt::ShiftModifier) {
        setPointSize(m_pointSize + (event->delta() > 0 ? 1 : -1));
        return;
    }

    QOpenGLWidget::wheelEvent(event);
}

void ZPointCloudWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->modifiers() & Qt::ControlModifier) {
        switch (event->key()) {
        case Qt::Key_Plus:
            qDebug() << "incrementing point size";
            setPointSize(m_pointSize + 1);
            return;
        case Qt::Key_Minus:
            qDebug() << "decrementing point size";
            setPointSize(m_pointSize - 1);
            return;
        }
    }

    QOpenGLWidget::keyPressEvent(event);
}
