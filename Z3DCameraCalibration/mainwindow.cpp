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

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "zcalibratedcamera.h"
#include "zcameracalibratorwidget.h"
#include "zcameraselectorwidget.h"

#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : Z3D::ZMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->finishButton->setVisible(false);

    QObject::connect(ui->continueButton, &QPushButton::clicked,
                     this, &MainWindow::onContinueButtonClicked);

    QObject::connect(ui->finishButton, &QPushButton::clicked,
                     this, &MainWindow::onFinishButtonClicked);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onCameraSelected(Z3D::ZCameraPtr camera)
{
    m_selectedCamera = camera;

    ui->finishButton->setVisible(m_selectedCamera.get());
}

void MainWindow::onContinueButtonClicked()
{
    if (ui->calibrateFromFilesRadioButton->isChecked()) {
        /// open the camera calibration window directly, without a camera
        m_selectedCamera = nullptr;
        onFinishButtonClicked();
    } else if (ui->calibrateOnlineRadioButton->isChecked()) {
        /// open the camera calibration window without a camera
        static Z3D::ZCameraSelectorWidget *cameraSelectorWidget = nullptr;
        if (!cameraSelectorWidget) {
            cameraSelectorWidget = new Z3D::ZCameraSelectorWidget(ui->pageCameraSelection);
            QObject::connect(cameraSelectorWidget, &Z3D::ZCameraSelectorWidget::cameraSelected,
                             this, &MainWindow::onCameraSelected);

            ui->cameraSelectionLayout->addWidget(cameraSelectorWidget);
        }
        ui->stackedWidget->setCurrentWidget(ui->pageCameraSelection);
    } else {
        qWarning() << "something is wrong!";
    }
}

void MainWindow::onFinishButtonClicked()
{
    if (m_selectedCamera) {
        Z3D::ZCalibratedCameraPtr calibratedCamera(new Z3D::ZCalibratedCamera(m_selectedCamera, nullptr));
        Z3D::ZCameraCalibratorWidget *calibratorWindow = new Z3D::ZCameraCalibratorWidget(calibratedCamera);
        calibratorWindow->show();
    } else {
        Z3D::ZCameraCalibratorWidget *calibratorWindow = new Z3D::ZCameraCalibratorWidget();
        calibratorWindow->show();
    }

    close();
    //deleteLater(); // we can't delete this, its the "main"!!
}
