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

#include "zcameraacquisition_fwd.h"
#include "zcameraacquisition_global.h"

#include <QDialog>

namespace Z3D
{

namespace Ui {
class ZCameraSelectorDialog;
}

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraSelectorDialog : public QDialog
{
    Q_OBJECT

public:
    ~ZCameraSelectorDialog();

    static Z3D::ZCameraPtr getCamera();

protected:
    explicit ZCameraSelectorDialog(QWidget *parent = nullptr);

protected slots:
    void onCameraSelected(const ZCameraPtr &camera);

private:
    Ui::ZCameraSelectorDialog *ui;

    Z3D::ZCameraPtr m_selectedCamera;
};

} // namespace Z3D
