/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "zsimplepointcloud.h"
#include "zstructuredlightsystem.h"

#include <QMainWindow>
#include <QPointer>

#include <opencv2/core/core.hpp>

namespace Ui {
class MainWindow;
}

namespace Z3D {
class ZPatternProjection;
}

class ZPointCloudWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:

private slots:
    void init();
    void initStructuredLightSystem();

    virtual void closeEvent(QCloseEvent *event);

    void onPatternProjectionTypeChanged(int index);

    void onScanFinished(Z3D::ZSimplePointCloud::Ptr cloud);

private:
    Ui::MainWindow *ui;

    QList<Z3D::ZPatternProjection *> m_patternProjectionList;
    Z3D::ZPatternProjection *m_currentPatternProjection;

    QList<Z3D::ZStructuredLightSystem *> m_structuredLightList;
    Z3D::ZStructuredLightSystem::Ptr m_currentStructuredLightSystem;

    ZPointCloudWidget *m_pointCloudWidget;
};

#endif // MAINWINDOW_H
