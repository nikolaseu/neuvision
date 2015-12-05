#include "zcameraselectorwidget.h"
#include "ui_zcameraselectorwidget.h"

#include "zcameraprovider.h"
#include "zcameraplugininterface.h"

namespace Z3D
{

ZCameraSelectorWidget::ZCameraSelectorWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZCameraSelectorWidget)
    , m_selectedCamera(0)
{
    ui->setupUi(this);

    m_pluginList = Z3D::ZCameraProvider::availablePlugins();

    foreach (Z3D::ZCameraPluginInterface *plugin, m_pluginList) {
        ui->pluginsComboBox->addItem(plugin->name());
    }

    QObject::connect(this, SIGNAL(cameraSelected(Z3D::ZCameraInterface::Ptr)),
                     ui->cameraPreview, SLOT(setCamera(Z3D::ZCameraInterface::Ptr)));

    QObject::connect(ui->pluginsComboBox, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(onPluginIndexChanged(int)));

    QObject::connect(ui->cameraListWidget, SIGNAL(currentRowChanged(int)),
                     this, SLOT(onCameraIndexChanged(int)));
}

ZCameraSelectorWidget::~ZCameraSelectorWidget()
{
    delete ui;
}

ZCameraInterface::Ptr ZCameraSelectorWidget::getCamera()
{
    Z3D::ZCameraSelectorWidget cameraSelector;
    cameraSelector.setWindowModality(Qt::WindowModal);
    cameraSelector.show();

    QEventLoop eventLoop;

    QObject::connect(&cameraSelector, SIGNAL(cameraSelected(Z3D::ZCameraInterface::Ptr)),
                     &eventLoop, SLOT(quit()));

    eventLoop.exec();

    return cameraSelector.m_selectedCamera;
}

void ZCameraSelectorWidget::onPluginIndexChanged(int index)
{
    ui->cameraListWidget->clear();

    Z3D::ZCameraPluginInterface *plugin = m_pluginList[index];

    m_currentCameraList = plugin->getConnectedCameras();

    foreach (Z3D::ZCameraInfo *cameraInfo, m_currentCameraList)
        ui->cameraListWidget->addItem(cameraInfo->name());
}

void ZCameraSelectorWidget::onCameraIndexChanged(int index)
{
    if (index >= 0 && index < m_currentCameraList.size()) {
        m_selectedCamera = m_currentCameraList[index]->getCamera();
    } else {
        m_selectedCamera = Z3D::ZCameraInterface::Ptr(0);
    }

    emit cameraSelected(m_selectedCamera);
}

Z3D::ZCameraInterface::Ptr ZCameraSelectorWidget::getSelectedCamera() const
{
    return m_selectedCamera;
}

} // namespace Z3D
