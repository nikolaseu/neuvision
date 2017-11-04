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

Z3D::ZChessboardCalibrationPatternFinderConfigWidget::ZChessboardCalibrationPatternFinderConfigWidget(ZChessboardCalibrationPatternFinder *patternFinder, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZChessboardCalibrationPatternFinderConfigWidget)
    , m_patternFinder(patternFinder)
{
    ui->setupUi(this);

    ui->sizeColumnsSpinBox->setValue(m_patternFinder->columns());
    QObject::connect(ui->sizeColumnsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setColumns);

    ui->sizeColWidthSpinBox->setValue(m_patternFinder->colWidth());
    QObject::connect(ui->sizeColWidthSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setColWidth);

    ui->sizeRowsSpinBox->setValue(m_patternFinder->rows());
    QObject::connect(ui->sizeRowsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setRows);

    ui->sizeRowHeightSpinBox->setValue(m_patternFinder->rowHeight());
    QObject::connect(ui->sizeRowHeightSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setRowHeight);

    ui->findCornersAdaptiveThreshCheckBox->setChecked(m_patternFinder->useAdaptiveThreshold());
    QObject::connect(ui->findCornersAdaptiveThreshCheckBox, &QCheckBox::toggled,
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setUseAdaptiveThreshold);

    ui->findCornersFastCheckCheckBox->setChecked(m_patternFinder->useFastCheck());
    QObject::connect(ui->findCornersFastCheckCheckBox, &QCheckBox::toggled,
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setUseFastCheck);

    ui->findCornersFilterQuadsCheckBox->setChecked(m_patternFinder->filterQuads());
    QObject::connect(ui->findCornersFilterQuadsCheckBox, &QCheckBox::toggled,
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setFilterQuads);

    ui->findCornersNormalizeImageCheckBox->setChecked(m_patternFinder->normalizeImage());
    QObject::connect(ui->findCornersNormalizeImageCheckBox, &QCheckBox::toggled,
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setNormalizeImage);

    ui->subpixWinWidthSpinBox->setValue(m_patternFinder->subPixWinWidth());
    QObject::connect(ui->subpixWinWidthSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setSubPixWinWidth);

    ui->subpixWinHeightSpinBox->setValue(m_patternFinder->subPixWinHeight());
    QObject::connect(ui->subpixWinHeightSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setSubPixWinHeight);

    ui->subpixZeroZoneWidthSpinBox->setValue(m_patternFinder->subPixZeroZoneWinWidth());
    QObject::connect(ui->subpixZeroZoneWidthSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setSubPixZeroZoneWinWidth);

    ui->subpixZeroZoneHeightSpinBox->setValue(m_patternFinder->subPixZeroZoneWinHeight());
    QObject::connect(ui->subpixZeroZoneHeightSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setSubPixZeroZoneWinHeight);

    ui->subpixMaxIterationsSpinBox->setValue(m_patternFinder->subPixMaxIter());
    QObject::connect(ui->subpixMaxIterationsSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setSubPixMaxIter);

    ui->subpixEpsilonSpinBox->setValue(m_patternFinder->subPixEpsilon());
    QObject::connect(ui->subpixEpsilonSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                     m_patternFinder, &ZChessboardCalibrationPatternFinder::setSubPixEpsilon);
}

Z3D::ZChessboardCalibrationPatternFinderConfigWidget::~ZChessboardCalibrationPatternFinderConfigWidget()
{
    delete ui;
}
