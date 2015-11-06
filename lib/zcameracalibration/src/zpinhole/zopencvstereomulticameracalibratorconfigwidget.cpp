#include "zopencvstereomulticameracalibratorconfigwidget.h"
#include "ui_zopencvstereomulticameracalibratorconfigwidget.h"

#include "zopencvstereomulticameracalibrator.h"

namespace Z3D
{

ZOpenCVStereoMultiCameraCalibratorConfigWidget::ZOpenCVStereoMultiCameraCalibratorConfigWidget(ZOpenCVStereoMultiCameraCalibrator *calibrator, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ZOpenCVStereoMultiCameraCalibratorConfigWidget),
    m_cameraCalibrator(calibrator)
{
    ui->setupUi(this);

    ui->fixIntrinsicCheckBox->setChecked(m_cameraCalibrator->fixIntrinsic());
    QObject::connect(m_cameraCalibrator, SIGNAL(fixIntrinsicChanged(bool)),
                     ui->fixIntrinsicCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->fixIntrinsicCheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setFixIntrinsic(bool)));

    ui->cameraMatrixGroupBox->setDisabled(m_cameraCalibrator->fixIntrinsic());
    QObject::connect(m_cameraCalibrator, SIGNAL(fixIntrinsicChanged(bool)),
                     ui->cameraMatrixGroupBox, SLOT(setDisabled(bool)));
    ui->distortionModelGroupBox->setDisabled(m_cameraCalibrator->fixIntrinsic());
    QObject::connect(m_cameraCalibrator, SIGNAL(fixIntrinsicChanged(bool)),
                     ui->distortionModelGroupBox, SLOT(setDisabled(bool)));

    ui->sameFocalLengthCheckBox->setChecked(m_cameraCalibrator->sameFocalLength());
    QObject::connect(m_cameraCalibrator, SIGNAL(sameFocalLengthChanged(bool)),
                     ui->sameFocalLengthCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->sameFocalLengthCheckBox, SIGNAL(toggled(bool)),
                     m_cameraCalibrator, SLOT(setSameFocalLength(bool)));

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

ZOpenCVStereoMultiCameraCalibratorConfigWidget::~ZOpenCVStereoMultiCameraCalibratorConfigWidget()
{
    delete ui;
}

} // namespace Z3D
