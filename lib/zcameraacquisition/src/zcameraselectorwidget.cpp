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

#include "ZCameraAcquisition/zcameraselectorwidget.h"
#include "ui_zcameraselectorwidget.h"

#include "ZCameraAcquisition/zcameraprovider.h"
#include "ZCameraAcquisition/zcameraplugininterface.h"
#include "ZCameraAcquisition/zcamerainfo.h"

namespace Z3D
{

ZCameraSelectorWidget::ZCameraSelectorWidget(QWidget *parent)
    : ZWidget(parent)
    , ui(new Ui::ZCameraSelectorWidget)
    , m_selectedCamera(nullptr)
{
    ui->setupUi(this);

    m_pluginList = ZCameraProvider::availablePlugins();

    for (auto *plugin : m_pluginList) {
        ui->pluginsComboBox->addItem(plugin->displayName());
    }

    QObject::connect(this, &ZCameraSelectorWidget::cameraSelected,
                     ui->cameraPreview, &ZCameraPreviewer::setCamera);

    QObject::connect(ui->pluginsComboBox, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                     this, &ZCameraSelectorWidget::onPluginIndexChanged);

    QObject::connect(ui->cameraListWidget, &QListWidget::currentRowChanged,
                     this, &ZCameraSelectorWidget::onCameraIndexChanged);
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

    for (const Z3D::ZCameraInfo *cameraInfo : m_currentCameraList) {
        ui->cameraListWidget->addItem(cameraInfo->name());
    }
}

void ZCameraSelectorWidget::onCameraIndexChanged(int index)
{
    if (index >= 0 && index < m_currentCameraList.size()) {
        m_selectedCamera = m_currentCameraList[index]->getCamera();
    } else {
        m_selectedCamera = nullptr;
    }

    emit cameraSelected(m_selectedCamera);
}

} // namespace Z3D
