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

#include "ZCameraCalibration/zopencvstereomulticameracalibratorconfigwidget.h"
#include "ui_zopencvstereomulticameracalibratorconfigwidget.h"

#include "ZCameraCalibration/zopencvstereomulticameracalibrator.h"

namespace Z3D
{

ZOpenCVStereoMultiCameraCalibratorConfigWidget::ZOpenCVStereoMultiCameraCalibratorConfigWidget(ZOpenCVStereoMultiCameraCalibrator *calibrator, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZOpenCVStereoMultiCameraCalibratorConfigWidget)
    , m_cameraCalibrator(calibrator)
{
    ui->setupUi(this);

    ui->fixIntrinsicCheckBox->setChecked(m_cameraCalibrator->fixIntrinsic());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::fixIntrinsicChanged,
                     ui->fixIntrinsicCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixIntrinsicCheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setFixIntrinsic);

    ui->cameraMatrixGroupBox->setDisabled(m_cameraCalibrator->fixIntrinsic());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::fixIntrinsicChanged,
                     ui->cameraMatrixGroupBox, &QGroupBox::setDisabled);
    ui->distortionModelGroupBox->setDisabled(m_cameraCalibrator->fixIntrinsic());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::fixIntrinsicChanged,
                     ui->distortionModelGroupBox, &QGroupBox::setDisabled);

    ui->sameFocalLengthCheckBox->setChecked(m_cameraCalibrator->sameFocalLength());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::sameFocalLengthChanged,
                     ui->sameFocalLengthCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->sameFocalLengthCheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setSameFocalLength);

    ui->zeroTangentialDistortionCheckBox->setChecked(m_cameraCalibrator->zeroTangentialDistortion());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::zeroTangentialDistortionChanged,
                     ui->zeroTangentialDistortionCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->zeroTangentialDistortionCheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setZeroTangentialDistortion);

    ui->useRationalModelCheckBox->setChecked(m_cameraCalibrator->useRationalModel());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::useRationalModelChanged,
                     ui->useRationalModelCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->useRationalModelCheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setUseRationalModel);

    ui->fixK1CheckBox->setChecked(m_cameraCalibrator->fixK1());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::fixK1Changed,
                     ui->fixK1CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK1CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setFixK1);

    ui->fixK2CheckBox->setChecked(m_cameraCalibrator->fixK2());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::fixK2Changed,
                     ui->fixK2CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK2CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setFixK2);

    ui->fixK3CheckBox->setChecked(m_cameraCalibrator->fixK3());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::fixK3Changed,
                     ui->fixK3CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK3CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setFixK3);

    ui->fixK4CheckBox->setChecked(m_cameraCalibrator->fixK4());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::fixK4Changed,
                     ui->fixK4CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK4CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setFixK4);

    ui->fixK5CheckBox->setChecked(m_cameraCalibrator->fixK5());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::fixK5Changed,
                     ui->fixK5CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK5CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setFixK5);

    ui->fixK6CheckBox->setChecked(m_cameraCalibrator->fixK6());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::fixK6Changed,
                     ui->fixK6CheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixK6CheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setFixK6);

    ui->fixPrincipalPointCheckBox->setChecked(m_cameraCalibrator->fixPrincipalPoint());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::fixPrincipalPointChanged,
                     ui->fixPrincipalPointCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixPrincipalPointCheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setFixPrincipalPoint);

    ui->fixAspectRatioCheckBox->setChecked(m_cameraCalibrator->fixAspectRatio());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::fixAspectRatioChanged,
                     ui->fixAspectRatioCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->fixAspectRatioCheckBox, &QCheckBox::toggled,
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setFixAspectRatio);

    ui->termMaxIterationsSpinBox->setValue(m_cameraCalibrator->termCriteriaMaxIterations());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::termCriteriaMaxIterationsChanged,
                     ui->termMaxIterationsSpinBox, &QSpinBox::setValue);
    QObject::connect(ui->termMaxIterationsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setTermCriteriaMaxIterations);

    ui->termEpsilonSpinBox->setValue(m_cameraCalibrator->termCriteriaEpsilon());
    QObject::connect(m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::termCriteriaEpsilonChanged,
                     ui->termEpsilonSpinBox, &QDoubleSpinBox::setValue);
    QObject::connect(ui->termEpsilonSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                     m_cameraCalibrator, &ZOpenCVStereoMultiCameraCalibrator::setTermCriteriaEpsilon);
}

ZOpenCVStereoMultiCameraCalibratorConfigWidget::~ZOpenCVStereoMultiCameraCalibratorConfigWidget()
{
    delete ui;
}

} // namespace Z3D
