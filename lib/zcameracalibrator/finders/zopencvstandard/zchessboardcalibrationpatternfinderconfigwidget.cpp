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

#include "zchessboardcalibrationpatternfinderconfigwidget.h"
#include "ui_zchessboardcalibrationpatternfinderconfigwidget.h"

#include "zchessboardcalibrationpatternfinder.h"

Z3D::ZChessboardCalibrationPatternFinderConfigWidget::ZChessboardCalibrationPatternFinderConfigWidget(ZChessboardCalibrationPatternFinder *patternFinder, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ZChessboardCalibrationPatternFinderConfigWidget),
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

    ui->findCornersAdaptiveThreshCheckBox->setChecked(m_patternFinder->useAdaptiveThreshold());
    QObject::connect(ui->findCornersAdaptiveThreshCheckBox, SIGNAL(toggled(bool)),
                     m_patternFinder, SLOT(setUseAdaptiveThreshold(bool)));

    ui->findCornersFastCheckCheckBox->setChecked(m_patternFinder->useFastCheck());
    QObject::connect(ui->findCornersFastCheckCheckBox, SIGNAL(toggled(bool)),
                     m_patternFinder, SLOT(setUseFastCheck(bool)));

    ui->findCornersFilterQuadsCheckBox->setChecked(m_patternFinder->filterQuads());
    QObject::connect(ui->findCornersFilterQuadsCheckBox, SIGNAL(toggled(bool)),
                     m_patternFinder, SLOT(setFilterQuads(bool)));

    ui->findCornersNormalizeImageCheckBox->setChecked(m_patternFinder->normalizeImage());
    QObject::connect(ui->findCornersNormalizeImageCheckBox, SIGNAL(toggled(bool)),
                     m_patternFinder, SLOT(setNormalizeImage(bool)));

    ui->subpixWinWidthSpinBox->setValue(m_patternFinder->subPixWinWidth());
    QObject::connect(ui->subpixWinWidthSpinBox, SIGNAL(valueChanged(int)),
                     m_patternFinder, SLOT(setSubPixWinWidth(int)));

    ui->subpixWinHeightSpinBox->setValue(m_patternFinder->subPixWinHeight());
    QObject::connect(ui->subpixWinHeightSpinBox, SIGNAL(valueChanged(int)),
                     m_patternFinder, SLOT(setSubPixWinHeight(int)));

    ui->subpixZeroZoneWidthSpinBox->setValue(m_patternFinder->subPixZeroZoneWinWidth());
    QObject::connect(ui->subpixZeroZoneWidthSpinBox, SIGNAL(valueChanged(int)),
                     m_patternFinder, SLOT(setSubPixZeroZoneWinWidth(int)));

    ui->subpixZeroZoneHeightSpinBox->setValue(m_patternFinder->subPixZeroZoneWinHeight());
    QObject::connect(ui->subpixZeroZoneHeightSpinBox, SIGNAL(valueChanged(int)),
                     m_patternFinder, SLOT(setSubPixZeroZoneWinHeight(int)));

    ui->subpixMaxIterationsSpinBox->setValue(m_patternFinder->subPixMaxIter());
    QObject::connect(ui->subpixMaxIterationsSpinBox, SIGNAL(valueChanged(int)),
                     m_patternFinder, SLOT(setSubPixMaxIter(int)));

    ui->subpixEpsilonSpinBox->setValue(m_patternFinder->subPixEpsilon());
    QObject::connect(ui->subpixEpsilonSpinBox, SIGNAL(valueChanged(double)),
                     m_patternFinder, SLOT(setSubPixEpsilon(double)));
}

Z3D::ZChessboardCalibrationPatternFinderConfigWidget::~ZChessboardCalibrationPatternFinderConfigWidget()
{
    delete ui;
}
