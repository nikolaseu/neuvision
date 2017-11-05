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

#include "zbinarypatternprojectionconfigwidget.h"
#include "ui_zbinarypatternprojectionconfigwidget.h"

#include "zbinarypatternprojection.h"

#include <QDesktopWidget>

namespace Z3D
{

ZBinaryPatternProjectionConfigWidget::ZBinaryPatternProjectionConfigWidget(ZBinaryPatternProjection *binaryPatternProjection, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZBinaryPatternProjectionConfigWidget)
    , m_binaryPatternProjection(binaryPatternProjection)
{
    ui->setupUi(this);

    /// use binary/gray binary
    QObject::connect(m_binaryPatternProjection, &ZBinaryPatternProjection::useGrayBinaryChanged,
                     ui->useGrayBinaryCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->useGrayBinaryCheckBox, &QCheckBox::toggled,
                     m_binaryPatternProjection, &ZBinaryPatternProjection::setUseGrayBinary);

    /// num patterns
    QObject::connect(m_binaryPatternProjection, &ZBinaryPatternProjection::numPatternsChanged,
                     ui->patternCountSpinBox, &QSpinBox::setValue);
    QObject::connect(ui->patternCountSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_binaryPatternProjection, &ZBinaryPatternProjection::setNumPatterns);

    /// current pattern
    QObject::connect(m_binaryPatternProjection, &ZBinaryPatternProjection::currentPatternChanged,
                     ui->currentPatternSpinBox, &QSpinBox::setValue);
    QObject::connect(ui->currentPatternSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_binaryPatternProjection, &ZBinaryPatternProjection::setCurrentPattern);

    /// vertical/horizontal
    QObject::connect(m_binaryPatternProjection, &ZBinaryPatternProjection::verticalChanged,
                     ui->verticalCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->verticalCheckBox, &QCheckBox::toggled,
                     m_binaryPatternProjection, &ZBinaryPatternProjection::setVertical);

    /// automatic pattern count
    QObject::connect(m_binaryPatternProjection, &ZBinaryPatternProjection::automaticPatternCountChanged,
                     ui->automaticPatternCountCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->automaticPatternCountCheckBox, &QCheckBox::toggled,
                     m_binaryPatternProjection, &ZBinaryPatternProjection::setAutomaticPatternCount);
    /// dependant of automatic pattern state
    QObject::connect(ui->automaticPatternCountCheckBox, &QCheckBox::toggled,
                     ui->patternCountSpinBox, &QSpinBox::setDisabled);
    QObject::connect(ui->automaticPatternCountCheckBox, &QCheckBox::toggled,
                     ui->patternCountLabel, &QLabel::setDisabled);

    /// inverted
    QObject::connect(m_binaryPatternProjection, &ZBinaryPatternProjection::invertedChanged,
                     ui->invertedCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->invertedCheckBox, &QCheckBox::toggled,
                     m_binaryPatternProjection, &ZBinaryPatternProjection::setInverted);

    /// inter photo delay
    QObject::connect(m_binaryPatternProjection, &ZBinaryPatternProjection::delayMsChanged,
                     ui->interPhotoDelaySpinBox, &QSpinBox::setValue);
    QObject::connect(ui->interPhotoDelaySpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_binaryPatternProjection, &ZBinaryPatternProjection::setDelayMs);

    /// noise threshold
    QObject::connect(m_binaryPatternProjection, &ZBinaryPatternProjection::noiseThresholdChanged,
                     ui->noiseThresholdSpinBox, &QSpinBox::setValue);
    QObject::connect(ui->noiseThresholdSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_binaryPatternProjection, &ZBinaryPatternProjection::setNoiseThreshold);

    /// intensity
    QObject::connect(m_binaryPatternProjection, &ZBinaryPatternProjection::intensityChanged,
                     ui->intensitySpinBox, &QDoubleSpinBox::setValue);
    QObject::connect(ui->intensitySpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                     m_binaryPatternProjection, &ZBinaryPatternProjection::setIntensity);

    /// debug mode
    QObject::connect(m_binaryPatternProjection, &ZBinaryPatternProjection::debugModeChanged,
                     ui->debugModeCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->debugModeCheckBox, &QCheckBox::toggled,
                     m_binaryPatternProjection, &ZBinaryPatternProjection::setDebugMode);

    /// preview
    QObject::connect(m_binaryPatternProjection, &ZBinaryPatternProjection::previewEnabledChanged,
                     ui->patternProjectionPreviewCheckBox, &QCheckBox::setChecked);
    QObject::connect(ui->patternProjectionPreviewCheckBox, &QCheckBox::toggled,
                     m_binaryPatternProjection, &ZBinaryPatternProjection::setPreviewEnabled, Qt::DirectConnection);
    /// dependant of preview state
    QObject::connect(ui->patternProjectionPreviewCheckBox, &QCheckBox::toggled,
                     ui->invertedCheckBox, &QCheckBox::setEnabled);
    QObject::connect(ui->patternProjectionPreviewCheckBox, &QCheckBox::toggled,
                     ui->currentPatternSpinBox, &QSpinBox::setEnabled);

    /// screens
    QObject::connect(ui->patternScreenComboBox, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                     this, &ZBinaryPatternProjectionConfigWidget::onSelectedScreenChanged);
    QObject::connect(QApplication::desktop(), &QDesktopWidget::screenCountChanged,
                     this, &ZBinaryPatternProjectionConfigWidget::updateScreens);
    QObject::connect(QApplication::desktop(), &QDesktopWidget::resized,
                     this, &ZBinaryPatternProjectionConfigWidget::updateScreens);
    QObject::connect(QApplication::desktop(), &QDesktopWidget::workAreaResized,
                     this, &ZBinaryPatternProjectionConfigWidget::updateScreens);
    updateScreens();

    /// set current values
    ui->useGrayBinaryCheckBox->setChecked(m_binaryPatternProjection->useGrayBinary());
    ui->automaticPatternCountCheckBox->setChecked(m_binaryPatternProjection->automaticPatternCount());
    ui->patternCountSpinBox->setValue(m_binaryPatternProjection->numPatterns());
    ui->patternProjectionPreviewCheckBox->setChecked(m_binaryPatternProjection->previewEnabled());
    ui->currentPatternSpinBox->setValue(m_binaryPatternProjection->currentPattern());
    ui->verticalCheckBox->setChecked(m_binaryPatternProjection->vertical());
    ui->invertedCheckBox->setChecked(m_binaryPatternProjection->inverted());
    ui->interPhotoDelaySpinBox->setValue(m_binaryPatternProjection->delayMs());
    ui->noiseThresholdSpinBox->setValue(m_binaryPatternProjection->noiseThreshold());
    ui->intensitySpinBox->setValue(m_binaryPatternProjection->intensity());
    ui->debugModeCheckBox->setChecked(m_binaryPatternProjection->debugMode());
}

ZBinaryPatternProjectionConfigWidget::~ZBinaryPatternProjectionConfigWidget()
{
    delete ui;
}

void ZBinaryPatternProjectionConfigWidget::onSelectedScreenChanged(int index)
{
    QDesktopWidget *desktop = QApplication::desktop();
    const QRect screenGeometry = desktop->screenGeometry(index);
    m_binaryPatternProjection->setProjectionWindowGeometry(screenGeometry);
}

void ZBinaryPatternProjectionConfigWidget::updateScreens()
{
    ui->patternScreenComboBox->blockSignals(true);

    ui->patternScreenComboBox->clear();

    ui->patternScreenComboBox->blockSignals(false);

    QDesktopWidget *desktop = QApplication::desktop();
    int screenCount = desktop->screenCount();
    int iPrimaryScreen = desktop->primaryScreen();

    /// add screens to combobox
    for (int i=0; i<screenCount; ++i) {
        const QRect screenGeometry = desktop->screenGeometry(i);
        QString text;
        if (i == iPrimaryScreen)
            text = QString("Screen %1 (%2x%3) [primary]");
        else
            text = QString("Screen %1 (%2x%3)");
        ui->patternScreenComboBox->addItem(text.arg(i+1).arg(screenGeometry.width()).arg(screenGeometry.height()));
    }

    //ui->patternScreenComboBox->blockSignals(false);

    /// if more than 1 screen is present, by default select the first "secondary"
    if (screenCount > 1) {
        int i;
        for (i=0; i<screenCount; ++i) {
            if (i != iPrimaryScreen)
                break;
        }
        ui->patternScreenComboBox->setCurrentIndex(i);
    }
}

} // namespace Z3D
