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

#include "zincompletecirclegridpatternfinderconfigwidget.h"
#include "ui_zincompletecirclegridpatternfinderconfigwidget.h"

#include "zincompletecirclegridpatternfinder.h"

namespace Z3D
{

ZIncompleteCircleGridPatternFinderConfigWidget::ZIncompleteCircleGridPatternFinderConfigWidget(
        ZIncompleteCircleGridPatternFinder *patternFinder,
        QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZIncompleteCircleGridPatternFinderConfigWidget)
    , m_patternFinder(patternFinder)
{
    ui->setupUi(this);

    ui->sizeColumnsSpinBox->setValue(m_patternFinder->columns());
    QObject::connect(ui->sizeColumnsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZIncompleteCircleGridPatternFinder::setColumns);

    ui->sizeColWidthSpinBox->setValue(m_patternFinder->colWidth());
    QObject::connect(ui->sizeColWidthSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                     m_patternFinder, &ZIncompleteCircleGridPatternFinder::setColWidth);

    ui->sizeRowsSpinBox->setValue(m_patternFinder->rows());
    QObject::connect(ui->sizeRowsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZIncompleteCircleGridPatternFinder::setRows);

    ui->sizeRowHeightSpinBox->setValue(m_patternFinder->rowHeight());
    QObject::connect(ui->sizeRowHeightSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                     m_patternFinder, &ZIncompleteCircleGridPatternFinder::setRowHeight);

    ui->maxSizeColumnsSpinBox->setValue(m_patternFinder->maxColumns());
    QObject::connect(ui->maxSizeColumnsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZIncompleteCircleGridPatternFinder::setMaxColumns);

    ui->maxSizeRowsSpinBox->setValue(m_patternFinder->maxRows());
    QObject::connect(ui->maxSizeRowsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZIncompleteCircleGridPatternFinder::setMaxRows);

    ui->asymmetricGridCheckBox->setChecked(m_patternFinder->isAsymmetricGrid());
    QObject::connect(ui->asymmetricGridCheckBox, &QCheckBox::toggled,
                     m_patternFinder, &ZIncompleteCircleGridPatternFinder::setIsAsymmetricGrid);

    ui->refinePatternPointsCheckBox->setChecked(m_patternFinder->refinePatternPoints());
    QObject::connect(ui->refinePatternPointsCheckBox, &QCheckBox::toggled,
                     m_patternFinder, &ZIncompleteCircleGridPatternFinder::setRefinePatternPoints);
}

ZIncompleteCircleGridPatternFinderConfigWidget::~ZIncompleteCircleGridPatternFinderConfigWidget()
{
    delete ui;
}

} // namespace Z3D
