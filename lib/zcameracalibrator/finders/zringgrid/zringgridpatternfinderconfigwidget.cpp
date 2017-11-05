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

ZRingGridPatternFinderConfigWidget::ZRingGridPatternFinderConfigWidget(ZRingGridPatternFinder *patternFinder, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZRingGridPatternFinderConfigWidget)
    , m_patternFinder(patternFinder)
{
    ui->setupUi(this);

    ui->sizeColumnsSpinBox->setValue(m_patternFinder->columns());
    QObject::connect(ui->sizeColumnsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZRingGridPatternFinder::setColumns);

    ui->sizeColWidthSpinBox->setValue(m_patternFinder->colWidth());
    QObject::connect(ui->sizeColWidthSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                     m_patternFinder, &ZRingGridPatternFinder::setColWidth);

    ui->sizeRowsSpinBox->setValue(m_patternFinder->rows());
    QObject::connect(ui->sizeRowsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZRingGridPatternFinder::setRows);

    ui->sizeRowHeightSpinBox->setValue(m_patternFinder->rowHeight());
    QObject::connect(ui->sizeRowHeightSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                     m_patternFinder, &ZRingGridPatternFinder::setRowHeight);

    ui->maxSizeColumnsSpinBox->setValue(m_patternFinder->maxColumns());
    QObject::connect(ui->maxSizeColumnsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZRingGridPatternFinder::setMaxColumns);

    ui->maxSizeRowsSpinBox->setValue(m_patternFinder->maxRows());
    QObject::connect(ui->maxSizeRowsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZRingGridPatternFinder::setMaxRows);

    ui->refinePatternPointsCheckBox->setChecked(m_patternFinder->refinePatternPoints());
    QObject::connect(ui->refinePatternPointsCheckBox, &QCheckBox::toggled,
                     m_patternFinder, &ZRingGridPatternFinder::setRefinePatternPoints);
}

ZRingGridPatternFinderConfigWidget::~ZRingGridPatternFinderConfigWidget()
{
    delete ui;
}

} // namespace Z3D
