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

} // namespace Z3D
