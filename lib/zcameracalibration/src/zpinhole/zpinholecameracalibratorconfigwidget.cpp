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

ZPinholeCameraCalibratorConfigWidget::ZPinholeCameraCalibratorConfigWidget(ZPinholeCameraCalibrator *calibrator, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ZPinholeCameraCalibratorConfigWidget),
    m_cameraCalibrator(calibrator)
{
    ui->setupUi(this);

    ui->zeroTangentialDistortionCheckBox->setChecked(m_cameraCalibrator->zeroTangentialDistortion());
    QObject::connect(m_cameraCalibrator, SIGNAL(zeroTangentialDistortionChanged(bool)),
                     ui->zeroTangentialDistortionCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->zeroTangentialDistortionCheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setZeroTangentialDistortion(bool)));

    ui->useRationalModelCheckBox->setChecked(m_cameraCalibrator->useRationalModel());
    QObject::connect(m_cameraCalibrator, SIGNAL(useRationalModelChanged(bool)),
                     ui->useRationalModelCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->useRationalModelCheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setUseRationalModel(bool)));

    ui->fixK1CheckBox->setChecked(m_cameraCalibrator->fixK1());
    QObject::connect(m_cameraCalibrator, SIGNAL(fixK1Changed(bool)),
                     ui->fixK1CheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->fixK1CheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setFixK1(bool)));

    ui->fixK2CheckBox->setChecked(m_cameraCalibrator->fixK2());
    QObject::connect(m_cameraCalibrator, SIGNAL(fixK2Changed(bool)),
                     ui->fixK2CheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->fixK2CheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setFixK2(bool)));

    ui->fixK3CheckBox->setChecked(m_cameraCalibrator->fixK3());
    QObject::connect(m_cameraCalibrator, SIGNAL(fixK3Changed(bool)),
                     ui->fixK3CheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->fixK3CheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setFixK3(bool)));

    ui->fixK4CheckBox->setChecked(m_cameraCalibrator->fixK4());
    QObject::connect(m_cameraCalibrator, SIGNAL(fixK4Changed(bool)),
                     ui->fixK4CheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->fixK4CheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setFixK4(bool)));

    ui->fixK5CheckBox->setChecked(m_cameraCalibrator->fixK5());
    QObject::connect(m_cameraCalibrator, SIGNAL(fixK5Changed(bool)),
                     ui->fixK5CheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->fixK5CheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setFixK5(bool)));

    ui->fixK6CheckBox->setChecked(m_cameraCalibrator->fixK6());
    QObject::connect(m_cameraCalibrator, SIGNAL(fixK6Changed(bool)),
                     ui->fixK6CheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->fixK6CheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setFixK6(bool)));

    ui->fixPrincipalPointCheckBox->setChecked(m_cameraCalibrator->fixPrincipalPoint());
    QObject::connect(m_cameraCalibrator, SIGNAL(fixPrincipalPointChanged(bool)),
                     ui->fixPrincipalPointCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->fixPrincipalPointCheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setFixPrincipalPoint(bool)));

    ui->fixAspectRatioCheckBox->setChecked(m_cameraCalibrator->fixAspectRatio());
    QObject::connect(m_cameraCalibrator, SIGNAL(fixAspectRatioChanged(bool)),
                     ui->fixAspectRatioCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->fixAspectRatioCheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setFixAspectRatio(bool)));

    ui->termMaxIterationsSpinBox->setValue(m_cameraCalibrator->termCriteriaMaxIterations());
    QObject::connect(m_cameraCalibrator, SIGNAL(termCriteriaMaxIterationsChanged(int)),
                     ui->termMaxIterationsSpinBox, SLOT(setValue(int)));
    QObject::connect(ui->termMaxIterationsSpinBox, SIGNAL(valueChanged(int)),
                     m_cameraCalibrator, SLOT(setTermCriteriaMaxIterations(int)));

    ui->termEpsilonSpinBox->setValue(m_cameraCalibrator->termCriteriaEpsilon());
    QObject::connect(m_cameraCalibrator, SIGNAL(termCriteriaEpsilonChanged(double)),
                     ui->termEpsilonSpinBox, SLOT(setValue(double)));
    QObject::connect(ui->termEpsilonSpinBox, SIGNAL(valueChanged(double)),
                     m_cameraCalibrator, SLOT(setTermCriteriaEpsilon(double)));
}

ZPinholeCameraCalibratorConfigWidget::~ZPinholeCameraCalibratorConfigWidget()
{
    delete ui;
}

} // namespace Z3D
