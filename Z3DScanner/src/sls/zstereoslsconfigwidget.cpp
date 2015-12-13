#include "zstereoslsconfigwidget.h"
#include "ui_zstereoslsconfigwidget.h"

#include "zstereosls.h"

namespace Z3D {

ZStereoSLSConfigWidget::ZStereoSLSConfigWidget(ZStereoSLS *stereoSLS, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZStereoSLSConfigWidget)
    , m_stereoSLS(stereoSLS)
{
    ui->setupUi(this);

    connect(m_stereoSLS, &ZStereoSLS::maxValidDistanceChanged,
            ui->maxValidDistanceSpinBox, &QDoubleSpinBox::setValue);
    connect(ui->maxValidDistanceSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            m_stereoSLS, &ZStereoSLS::setMaxValidDistance);

    connect(m_stereoSLS, &ZStereoSLS::debugShowDecodedImagesChanged,
            ui->debugShowDecodedImagesCheckBox, &QCheckBox::setChecked);
    connect(ui->debugShowDecodedImagesCheckBox, &QCheckBox::toggled,
            m_stereoSLS, &ZStereoSLS::setDebugShowDecodedImages);

    connect(m_stereoSLS, &ZStereoSLS::debugShowFringesChanged,
            ui->debugShowFringesCheckBox, &QCheckBox::setChecked);
    connect(ui->debugShowFringesCheckBox, &QCheckBox::toggled,
            m_stereoSLS, &ZStereoSLS::setDebugShowFringes);

    ui->maxValidDistanceSpinBox->setValue(m_stereoSLS->maxValidDistance());
    ui->debugShowDecodedImagesCheckBox->setChecked(m_stereoSLS->debugShowDecodedImages());
    ui->debugShowFringesCheckBox->setChecked(m_stereoSLS->debugShowFringes());
}

ZStereoSLSConfigWidget::~ZStereoSLSConfigWidget()
{
    delete ui;
}

} // namespace Z3D
