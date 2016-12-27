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

#include "zlasertriangulation_global.h"
#include "zcamera3dinterface.h"

#include <QObject>

namespace Z3D
{

class Z3D_LASERTRIANGULATION_SHARED_EXPORT PointCloudRecorder : public QObject
{
    Q_OBJECT

public:
    typedef QSharedPointer<Z3D::PointCloudRecorder> Ptr;

    explicit PointCloudRecorder(QObject *parent = 0);

    Camera3DInterface::WeakPtr getCamera() const;
    void setCamera(Camera3DInterface::WeakPtr camera);

signals:

public slots:
    void setEnabled(bool enabled);

    void onAcquisitionStarted();
    void onAcquisitionStopped();

    void onNewPointCloudReceived(Z3D::ZPointCloud::Ptr cloud);

private:
    Camera3DInterface::WeakPtr m_camera;

    bool m_enabled;

    QString m_basePath;
    QString m_currentSavePath;
};

} // namespace Z3D
