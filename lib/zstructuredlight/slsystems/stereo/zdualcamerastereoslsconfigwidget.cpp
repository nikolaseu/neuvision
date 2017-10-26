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

#include "zdualcamerastereoslsconfigwidget.h"
#include "ui_zdualcamerastereoslsconfigwidget.h"

#include "zcameracalibratorwidget.h"
#include "zcamerapreviewer.h"
#include "zmulticameracalibratorwidget.h"
#include "zdualcamerastereosls.h"

#include <QMenu>

namespace Z3D {

ZDualCameraStereoSLSConfigWidget::ZDualCameraStereoSLSConfigWidget(ZDualCameraStereoSLS *stereoSLS, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZDualCameraStereoSLSConfigWidget)
    , m_stereoSLS(stereoSLS)
{
    ui->setupUi(this);

    /// Update UI
    connect(ui->calibrateSystemButton, &QPushButton::clicked, [&](){
        std::vector<Z3D::ZCalibratedCamera::Ptr> cameras;
        cameras.push_back(m_stereoSLS->leftCamera());
        cameras.push_back(m_stereoSLS->rightCamera());
        Z3D::ZMultiCameraCalibratorWidget *calibWidget = new Z3D::ZMultiCameraCalibratorWidget(cameras);
        calibWidget->show();
    });

    connect(m_stereoSLS, &ZDualCameraStereoSLS::maxValidDistanceChanged,
            ui->maxValidDistanceSpinBox, &QDoubleSpinBox::setValue);
    connect(ui->maxValidDistanceSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            m_stereoSLS, &ZDualCameraStereoSLS::setMaxValidDistance);

    connect(m_stereoSLS, &ZDualCameraStereoSLS::debugShowDecodedImagesChanged,
            ui->debugShowDecodedImagesCheckBox, &QCheckBox::setChecked);
    connect(ui->debugShowDecodedImagesCheckBox, &QCheckBox::toggled,
            m_stereoSLS, &ZDualCameraStereoSLS::setDebugShowDecodedImages);

    ui->maxValidDistanceSpinBox->setValue(m_stereoSLS->maxValidDistance());
    ui->debugShowDecodedImagesCheckBox->setChecked(m_stereoSLS->debugShowDecodedImages());

    /// Left camera
    QMenu *leftCameraMenu = new QMenu(this);
    QAction *leftPreviewAction = leftCameraMenu->addAction(tr("Preview..."));
    connect(leftPreviewAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->leftCamera()->camera();
        auto *previewDialog = new Z3D::ZCameraPreviewer(camera);
        previewDialog->show();
    });
    QAction *leftSettingsAction = leftCameraMenu->addAction(tr("Settings..."));
    connect(leftSettingsAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->leftCamera()->camera();
        camera->showSettingsDialog();
    });
    QAction *leftCalibrationAction = leftCameraMenu->addAction(tr("Calibration..."));
    connect(leftCalibrationAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->leftCamera();
        auto *calibrator = new Z3D::ZCameraCalibratorWidget(camera);
        calibrator->show();
    });
    ui->leftCameraButton->setMenu(leftCameraMenu);

    /// Right camera
    QMenu *rightCameraMenu = new QMenu(this);
    QAction *rightPreviewAction = rightCameraMenu->addAction(tr("Preview..."));
    connect(rightPreviewAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->rightCamera()->camera();
        auto *previewDialog = new Z3D::ZCameraPreviewer(camera);
        previewDialog->show();
    });
    QAction *rightSettingsAction = rightCameraMenu->addAction(tr("Settings..."));
    connect(rightSettingsAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->rightCamera()->camera();
        camera->showSettingsDialog();
    });
    QAction *rightCalibrationAction = rightCameraMenu->addAction(tr("Calibration..."));
    connect(rightCalibrationAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->rightCamera();
        auto *calibrator = new Z3D::ZCameraCalibratorWidget(camera);
        calibrator->show();
    });
    ui->rightCameraButton->setMenu(rightCameraMenu);
}

ZDualCameraStereoSLSConfigWidget::~ZDualCameraStereoSLSConfigWidget()
{
    delete ui;
}

} // namespace Z3D
