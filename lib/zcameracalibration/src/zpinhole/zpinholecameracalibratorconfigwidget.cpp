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

#include "zpinholecameracalibratorconfigwidget.h"
#include "ui_zpinholecameracalibratorconfigwidget.h"

#include "zpinholecameracalibrator.h"

namespace Z3D
{

ZPinholeCameraCalibratorConfigWidget::ZPinholeCameraCalibratorConfigWidget(ZPinholeCameraCalibrator *calibrator, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZPinholeCameraCalibratorConfigWidget)
    , m_cameraCalibrator(calibrator)
{
    ui->setupUi(this);

    ui->zeroTangentialDistortionCheckBox->setChecked(m_cameraCalibrator->zeroTangentialDistortion());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::zeroTangentialDistortionChanged,
                     ui->zeroTangentialDistortionCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->zeroTangentialDistortionCheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setZeroTangentialDistortion);

    ui->useRationalModelCheckBox->setChecked(m_cameraCalibrator->useRationalModel());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::useRationalModelChanged,
                     ui->useRationalModelCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->useRationalModelCheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setUseRationalModel);

    ui->fixK1CheckBox->setChecked(m_cameraCalibrator->fixK1());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::fixK1Changed,
                     ui->fixK1CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK1CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setFixK1);

    ui->fixK2CheckBox->setChecked(m_cameraCalibrator->fixK2());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::fixK2Changed,
                     ui->fixK2CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK2CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setFixK2);

    ui->fixK3CheckBox->setChecked(m_cameraCalibrator->fixK3());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::fixK3Changed,
                     ui->fixK3CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK3CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setFixK3);

    ui->fixK4CheckBox->setChecked(m_cameraCalibrator->fixK4());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::fixK4Changed,
                     ui->fixK4CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK4CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setFixK4);

    ui->fixK5CheckBox->setChecked(m_cameraCalibrator->fixK5());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::fixK5Changed,
                     ui->fixK5CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK5CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setFixK5);

    ui->fixK6CheckBox->setChecked(m_cameraCalibrator->fixK6());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::fixK6Changed,
                     ui->fixK6CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK6CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setFixK6);

    ui->fixPrincipalPointCheckBox->setChecked(m_cameraCalibrator->fixPrincipalPoint());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::fixPrincipalPointChanged,
                     ui->fixPrincipalPointCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixPrincipalPointCheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setFixPrincipalPoint);

    ui->fixAspectRatioCheckBox->setChecked(m_cameraCalibrator->fixAspectRatio());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::fixAspectRatioChanged,
                     ui->fixAspectRatioCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixAspectRatioCheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setFixAspectRatio);

    ui->termMaxIterationsSpinBox->setValue(m_cameraCalibrator->termCriteriaMaxIterations());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::termCriteriaMaxIterationsChanged,
                     ui->termMaxIterationsSpinBox, &QSpinBox::setValue);
    QObject::connect(ui->termMaxIterationsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setTermCriteriaMaxIterations);

    ui->termEpsilonSpinBox->setValue(m_cameraCalibrator->termCriteriaEpsilon());
    QObject::connect(m_cameraCalibrator, &ZPinholeCameraCalibrator::termCriteriaEpsilonChanged,
                     ui->termEpsilonSpinBox, &QDoubleSpinBox::setValue);
    QObject::connect(ui->termEpsilonSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                     m_cameraCalibrator, &ZPinholeCameraCalibrator::setTermCriteriaEpsilon);
}

ZPinholeCameraCalibratorConfigWidget::~ZPinholeCameraCalibratorConfigWidget()
{
    delete ui;
}

} // namespace Z3D
