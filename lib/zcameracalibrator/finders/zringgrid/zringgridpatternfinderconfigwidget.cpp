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

#include "zringgridpatternfinderconfigwidget.h"
#include "ui_zringgridpatternfinderconfigwidget.h"

#include "zringgridpatternfinder.h"

namespace Z3D
{

ZRingGridPatternFinderConfigWidget::ZRingGridPatternFinderConfigWidget(ZRingGridPatternFinder *patternFinder, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ZRingGridPatternFinderConfigWidget),
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

    ui->refinePatternPointsCheckBox->setChecked(m_patternFinder->refinePatternPoints());
    QObject::connect(ui->refinePatternPointsCheckBox, SIGNAL(toggled(bool)),
                     m_patternFinder, SLOT(setRefinePatternPoints(bool)));
}

ZRingGridPatternFinderConfigWidget::~ZRingGridPatternFinderConfigWidget()
{
    delete ui;
}

} // namespace Z3D
