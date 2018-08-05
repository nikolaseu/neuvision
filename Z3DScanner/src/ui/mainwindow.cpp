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

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "zpatternprojection.h"
#include "zpatternprojectionprovider.h"
#include "zstructuredlightsystem.h"
#include "zstructuredlightsystemprovider.h"

#include "src/zpointcloudwidget.h"

#include <QDebug>
#include <QMessageBox>



MainWindow::MainWindow(Z3D::ZStructuredLightSystemPtr structuredLightSystem, QWidget *parent)
    : Z3D::ZMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_currentStructuredLightSystem(structuredLightSystem)
{
    ui->setupUi(this);

    m_pointCloudWidget = new ZPointCloudWidget(this);
    ui->glLayout->addWidget(m_pointCloudWidget);

    /// disable all until ready
    ui->centralwidget->setEnabled(m_currentStructuredLightSystem->ready());

    /// finish initialization
    connect(m_currentStructuredLightSystem.get(), &Z3D::ZStructuredLightSystem::readyChanged,
            ui->centralwidget, &QWidget::setEnabled);
    connect(m_currentStructuredLightSystem.get(), &Z3D::ZStructuredLightSystem::scanFinished,
            this, &MainWindow::onScanFinished);
    connect(ui->startAcquisitionButton, &QPushButton::clicked,
            m_currentStructuredLightSystem.get(), &Z3D::ZStructuredLightSystem::start);

    /// add config widget
    QWidget *currentWidget = Z3D::ZStructuredLightSystemProvider::getConfigWidget(m_currentStructuredLightSystem.get());
    ui->structuredLightSystemConfigLayout->addWidget(currentWidget);
    currentWidget->setVisible(true);

    ui->structuredLightSystemLabel->setText(m_currentStructuredLightSystem->metaObject()->className());

    /// add pattern projection config widget
    auto m_currentPatternProjection = m_currentStructuredLightSystem->patternProjection();
    ui->patternTypeLabel->setText(m_currentPatternProjection->metaObject()->className());

    QWidget *currentWidget2 = Z3D::ZPatternProjectionProvider::getConfigWidget(m_currentPatternProjection.get());
    ui->patternProjectionConfigLayout->addWidget(currentWidget2);
    currentWidget2->setVisible(true);
}

MainWindow::~MainWindow()
{
    qDebug() << "deleting ui...";
    delete ui;

    qDebug() << Q_FUNC_INFO << "finished";
}

void MainWindow::closeEvent(QCloseEvent * /*event*/)
{
    /// closing the main window causes the program to quit
    qApp->quit();
}

void MainWindow::onScanFinished(Z3D::ZPointCloudPtr cloud)
{
    if (!cloud) {
        QMessageBox::warning(this, tr("Scan error"),
                             tr("The point cloud is empty.\nIs everything configured correctly?\nIs there anything inside the scan volume?"));
        return;
    }

    m_pointCloudWidget->setPointCloud(cloud);
}
