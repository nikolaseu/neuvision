#include "zincompletecirclegridpatternfinderconfigwidget.h"
#include "ui_zincompletecirclegridpatternfinderconfigwidget.h"

#include "zincompletecirclegridpatternfinder.h"

namespace Z3D
{

ZIncompleteCircleGridPatternFinderConfigWidget::ZIncompleteCircleGridPatternFinderConfigWidget(ZIncompleteCircleGridPatternFinder *patternFinder, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ZIncompleteCircleGridPatternFinderConfigWidget),
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

    ui->maxSizeColumnsSpinBox->setValue(m_patternFinder->maxColumns());
    QObject::connect(ui->maxSizeColumnsSpinBox, SIGNAL(valueChanged(int)),
                     m_patternFinder, SLOT(setMaxColumns(int)));

    ui->maxSizeRowsSpinBox->setValue(m_patternFinder->maxRows());
    QObject::connect(ui->maxSizeRowsSpinBox, SIGNAL(valueChanged(int)),
                     m_patternFinder, SLOT(setMaxRows(int)));

    ui->asymmetricGridCheckBox->setChecked(m_patternFinder->isAsymmetricGrid());
    QObject::connect(ui->asymmetricGridCheckBox, SIGNAL(toggled(bool)),
                     m_patternFinder, SLOT(setIsAsymmetricGrid(bool)));

    ui->refinePatternPointsCheckBox->setChecked(m_patternFinder->refinePatternPoints());
    QObject::connect(ui->refinePatternPointsCheckBox, SIGNAL(toggled(bool)),
                     m_patternFinder, SLOT(setRefinePatternPoints(bool)));
}

ZIncompleteCircleGridPatternFinderConfigWidget::~ZIncompleteCircleGridPatternFinderConfigWidget()
{
    delete ui;
}

} // namespace Z3D
