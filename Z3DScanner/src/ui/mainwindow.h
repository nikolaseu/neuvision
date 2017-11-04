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

#pragma once

#include "zmainwindow.h"

#include <Z3DStructuredLight>

#include <QPointer>

#include <opencv2/core.hpp>

namespace Ui {
class MainWindow;
}

class ZPointCloudWidget;

class MainWindow : public Z3D::ZMainWindow
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

    void onScanFinished(Z3D::ZSimplePointCloudPtr cloud);

private:
    Ui::MainWindow *ui;

    std::vector<Z3D::ZPatternProjectionPtr> m_patternProjectionList;
    Z3D::ZPatternProjectionPtr m_currentPatternProjection;

    QList<Z3D::ZStructuredLightSystem *> m_structuredLightList;
    Z3D::ZStructuredLightSystemPtr m_currentStructuredLightSystem;

    ZPointCloudWidget *m_pointCloudWidget;
};
