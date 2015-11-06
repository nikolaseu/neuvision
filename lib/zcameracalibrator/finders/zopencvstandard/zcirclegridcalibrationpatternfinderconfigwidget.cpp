#include "zcirclegridcalibrationpatternfinderconfigwidget.h"
#include "ui_zcirclegridcalibrationpatternfinderconfigwidget.h"

#include "zcirclegridcalibrationpatternfinder.h"

namespace Z3D
{

ZCircleGridCalibrationPatternFinderConfigWidget::ZCircleGridCalibrationPatternFinderConfigWidget(ZCircleGridCalibrationPatternFinder *patternFinder, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ZCircleGridCalibrationPatternFinderConfigWidget),
    m_patternFinder(patternFinder)
{
    ui->setupUi(this);

    ui->sizeColumnsSpinBox->setValue(m_patternFinder->columns());
    QObject::connect(ui->sizeColumnsSpinBox, SIGNAL(valueChanged(int)),
                     m_patternFinder, SLOT(setColumns(int)));

    ui->sizeColWidthSpinBox->setValue(m_patternFinder->colWidth());
    QObject::connect(ui->sizeColWidthSpinBox, SIGNAL(valueChanged(double)),
                     m_patternFinder, SLOT(setColWidth(double)));

    ui->sizeRowsSpinBox->setValue(m_patternFinder->rows());
    QObject::connect(ui->sizeRowsSpinBox, SIGNAL(valueChanged(int)),
                     m_patternFinder, SLOT(setRows(int)));

    ui->sizeRowHeightSpinBox->setValue(m_patternFinder->rowHeight());
    QObject::connect(ui->sizeRowHeightSpinBox, SIGNAL(valueChanged(double)),
                     m_patternFinder, SLOT(setRowHeight(double)));
}

ZCircleGridCalibrationPatternFinderConfigWidget::~ZCircleGridCalibrationPatternFinderConfigWidget()
{
    delete ui;
}

} // namespace Z3D
