/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2018 Nicolas Ulrich <nikolaseu@gmail.com>
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

#include "ZPointCloud/zpointcloud_fwd.h"
#include "ZPointCloud/zpointcloud_global.h"

#include <QObject>
#include <QUrl>

namespace Z3D
{

class ZPointCloud;

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZPointCloudReader : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString filename READ filename WRITE setFilename NOTIFY filenameChanged)
    Q_PROPERTY(QUrl source READ source WRITE setSource NOTIFY sourceChanged)
    Q_PROPERTY(Z3D::ZPointCloud *pointCloud READ pointCloud NOTIFY pointCloudChanged)

public:
    explicit ZPointCloudReader();

    QString filename() const;
    QUrl source() const;
    ZPointCloud *pointCloud() const;

public slots:
    void setFilename(const QString &filename);
    void setSource(const QUrl &source);

signals:
    void filenameChanged(QString filename);
    void pointCloudChanged(Z3D::ZPointCloud *pointcloud);
    void sourceChanged(QUrl source);

private:
    QString m_filename;
    QUrl m_source;
    ZPointCloudPtr m_pointCloud = nullptr;
};

} // namespace Z3D
