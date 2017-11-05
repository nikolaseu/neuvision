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

#include "zscannerinitialconfigwizard.h"
#include "ui_zscannerinitialconfigwizard.h"

#include "zcameraselectorwidget.h"

#include <QDebug>

ZScannerInitialConfigWizard::ZScannerInitialConfigWizard(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZScannerInitialConfigWizard)
{
    ui->setupUi(this);
}

ZScannerInitialConfigWizard::~ZScannerInitialConfigWizard()
{
    delete ui;
}

void ZScannerInitialConfigWizard::on_pushButton_clicked()
{
    int currentPage =  ui->stackedWidget->currentIndex();
    switch (currentPage) {
    case 0: {
        static Z3D::ZCameraSelectorWidget *m_leftCameraSelectorWidget = nullptr;
        if (!m_leftCameraSelectorWidget) {
            m_leftCameraSelectorWidget = new Z3D::ZCameraSelectorWidget(ui->pageLeftCameraSelection);
            //QObject::connect(m_leftCameraSelectorWidget, SIGNAL(cameraSelected(Z3D::ZCameraInterfacePtr)),
            //                 this, SLOT(onCameraSelected(Z3D::ZCameraInterfacePtr)));

            QVBoxLayout *layout = new QVBoxLayout(ui->pageLeftCameraSelection);
            layout->setMargin(0);
            layout->addWidget(m_leftCameraSelectorWidget);

            ui->pageLeftCameraSelection->setLayout(layout);
        }

        ui->titleLabel->setText(tr("Left camera selection"));
        ui->stackedWidget->setCurrentIndex(currentPage+1);
        break;
    }
    case 1:{
        static Z3D::ZCameraSelectorWidget *m_rightCameraSelectorWidget = nullptr;
        if (!m_rightCameraSelectorWidget) {
            m_rightCameraSelectorWidget = new Z3D::ZCameraSelectorWidget(ui->pageRightCameraSelection);
            //QObject::connect(m_rightCameraSelectorWidget, SIGNAL(cameraSelected(Z3D::ZCameraInterfacePtr)),
            //                 this, SLOT(onCameraSelected(Z3D::ZCameraInterfacePtr)));

            QVBoxLayout *layout = new QVBoxLayout(ui->pageRightCameraSelection);
            layout->setMargin(0);
            layout->addWidget(m_rightCameraSelectorWidget);

            ui->pageRightCameraSelection->setLayout(layout);
        }

        ui->titleLabel->setText(tr("Right camera selection"));
        ui->stackedWidget->setCurrentIndex(currentPage+1);
        break;
    }
    case 2:{
        ui->titleLabel->setText(tr("Intrinsic camera calibration"));
        ui->stackedWidget->setCurrentIndex(currentPage+1);
        break;
    }
    case 3:{
        ui->titleLabel->setText(tr("Extrinsic camera calibration"));
        ui->stackedWidget->setCurrentIndex(currentPage+1);
        break;
    }
    case 4:
        ui->titleLabel->setText(tr("ZScanner configuration"));
        ui->stackedWidget->setCurrentIndex(0);
        break;
    default:
        qWarning() << "some index missing?" << currentPage;
    }
}
