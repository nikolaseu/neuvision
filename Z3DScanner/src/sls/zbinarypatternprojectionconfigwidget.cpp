#include "zbinarypatternprojectionconfigwidget.h"
#include "ui_zbinarypatternprojectionconfigwidget.h"

#include "zbinarypatternprojection.h"

#include <QDesktopWidget>

namespace Z3D
{

ZBinaryPatternProjectionConfigWidget::ZBinaryPatternProjectionConfigWidget(
        ZBinaryPatternProjection *binaryPatternProjection, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ZBinaryPatternProjectionConfigWidget),
    m_binaryPatternProjection(binaryPatternProjection)
{
    ui->setupUi(this);

    /// use binary/gray binary
    QObject::connect(m_binaryPatternProjection, SIGNAL(useGrayBinaryChanged(bool)),
                     ui->useGrayBinaryCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->useGrayBinaryCheckBox, SIGNAL(toggled(bool)),
                     m_binaryPatternProjection, SLOT(setUseGrayBinary(bool)));

    /// num patterns
    QObject::connect(m_binaryPatternProjection, SIGNAL(numPatternsChanged(int)),
                     ui->patternCountSpinBox, SLOT(setValue(int)));
    QObject::connect(ui->patternCountSpinBox, SIGNAL(valueChanged(int)),
                     m_binaryPatternProjection, SLOT(setNumPatterns(int)));

    /// current pattern
    QObject::connect(m_binaryPatternProjection, SIGNAL(currentPatternChanged(int)),
                     ui->currentPatternSpinBox, SLOT(setValue(int)));
    QObject::connect(ui->currentPatternSpinBox, SIGNAL(valueChanged(int)),
                     m_binaryPatternProjection, SLOT(setCurrentPattern(int)));

    /// vertical/horizontal
    QObject::connect(m_binaryPatternProjection, SIGNAL(verticalChanged(bool)),
                     ui->verticalCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->verticalCheckBox, SIGNAL(toggled(bool)),
                     m_binaryPatternProjection, SLOT(setVertical(bool)));

    /// automatic pattern count
    QObject::connect(m_binaryPatternProjection, SIGNAL(automaticPatternCountChanged(bool)),
                     ui->automaticPatternCountCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->automaticPatternCountCheckBox, SIGNAL(toggled(bool)),
                     m_binaryPatternProjection, SLOT(setAutomaticPatternCount(bool)));
    /// dependant of automatic pattern state
    QObject::connect(ui->automaticPatternCountCheckBox, SIGNAL(toggled(bool)),
                     ui->patternCountSpinBox, SLOT(setDisabled(bool)));
    QObject::connect(ui->automaticPatternCountCheckBox, SIGNAL(toggled(bool)),
                     ui->patternCountLabel, SLOT(setDisabled(bool)));

    /// inverted
    QObject::connect(m_binaryPatternProjection, SIGNAL(invertedChanged(bool)),
                     ui->invertedCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->invertedCheckBox, SIGNAL(toggled(bool)),
                     m_binaryPatternProjection, SLOT(setInverted(bool)));

    /// inter photo delay
    QObject::connect(m_binaryPatternProjection, SIGNAL(delayMsChanged(int)),
                     ui->interPhotoDelaySpinBox, SLOT(setValue(int)));
    QObject::connect(ui->interPhotoDelaySpinBox, SIGNAL(valueChanged(int)),
                     m_binaryPatternProjection, SLOT(setDelayMs(int)));

    /// noise threshold
    QObject::connect(m_binaryPatternProjection, SIGNAL(noiseThresholdChanged(int)),
                     ui->noiseThresholdSpinBox, SLOT(setValue(int)));
    QObject::connect(ui->noiseThresholdSpinBox, SIGNAL(valueChanged(int)),
                     m_binaryPatternProjection, SLOT(setNoiseThreshold(int)));

    /// intensity
    QObject::connect(m_binaryPatternProjection, SIGNAL(intensityChanged(double)),
                     ui->intensitySpinBox, SLOT(setValue(double)));
    QObject::connect(ui->intensitySpinBox, SIGNAL(valueChanged(double)),
                     m_binaryPatternProjection, SLOT(setIntensity(double)));

    /// debug mode
    QObject::connect(m_binaryPatternProjection, SIGNAL(debugModeChanged(bool)),
                     ui->debugModeCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->debugModeCheckBox, SIGNAL(toggled(bool)),
                     m_binaryPatternProjection, SLOT(setDebugMode(bool)));

    /// preview
    QObject::connect(m_binaryPatternProjection, SIGNAL(previewEnabledChanged(bool)),
                     ui->patternProjectionPreviewCheckBox, SLOT(setChecked(bool)));
    QObject::connect(ui->patternProjectionPreviewCheckBox, SIGNAL(toggled(bool)),
                     m_binaryPatternProjection, SLOT(setPreviewEnabled(bool)), Qt::DirectConnection);
    /// dependant of preview state
    QObject::connect(ui->patternProjectionPreviewCheckBox, SIGNAL(toggled(bool)),
                     ui->invertedCheckBox, SLOT(setEnabled(bool)));
    QObject::connect(ui->patternProjectionPreviewCheckBox, SIGNAL(toggled(bool)),
                     ui->currentPatternSpinBox, SLOT(setEnabled(bool)));

    /// screens
    QObject::connect(ui->patternScreenComboBox, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(onSelectedScreenChanged(int)));
    QObject::connect(QApplication::desktop(), SIGNAL(screenCountChanged(int)),
                     this, SLOT(updateScreens()));
    QObject::connect(QApplication::desktop(), SIGNAL(resized(int)),
                     this, SLOT(updateScreens()));
    QObject::connect(QApplication::desktop(), SIGNAL(workAreaResized(int)),
                     this, SLOT(updateScreens()));
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
