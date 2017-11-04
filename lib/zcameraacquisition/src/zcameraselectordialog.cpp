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

#include "zcameraselectordialog.h"
#include "ui_zcameraselectordialog.h"

namespace Z3D
{

ZCameraSelectorDialog::ZCameraSelectorDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ZCameraSelectorDialog)
{
    ui->setupUi(this);

    ui->continueButton->setVisible(false);

    QObject::connect(ui->cameraSelectorWidget, SIGNAL(cameraSelected(Z3D::ZCameraInterface::Ptr)),
                     this, SLOT(onCameraSelected(Z3D::ZCameraInterface::Ptr)));
}

ZCameraSelectorDialog::~ZCameraSelectorDialog()
{
    delete ui;
}

Z3D::ZCameraInterface::Ptr ZCameraSelectorDialog::getCamera() {
    Z3D::ZCameraSelectorDialog cameraSelector;
    cameraSelector.setWindowModality(Qt::WindowModal);
    cameraSelector.show();

    QEventLoop eventLoop;

    QObject::connect(cameraSelector.ui->continueButton, SIGNAL(clicked(bool)),
                     &eventLoop, SLOT(quit()));
    QObject::connect(&cameraSelector, SIGNAL(finished(int)),
                     &eventLoop, SLOT(quit()));

    eventLoop.exec();

    return cameraSelector.m_selectedCamera;
}

void ZCameraSelectorDialog::onCameraSelected(Z3D::ZCameraInterface::Ptr camera)
{
    m_selectedCamera = camera;

    ui->continueButton->setVisible(m_selectedCamera.get());
}

} // namespace Z3D
