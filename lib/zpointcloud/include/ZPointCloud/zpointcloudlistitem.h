/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2019 Nicolas Ulrich <nikolaseu@gmail.com>
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

#include <QtCore/QObject>
#include <QtGui/QMatrix4x4>

namespace Z3D
{

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZPointCloudListItem : public QObject
{
    Q_OBJECT

    Q_PROPERTY(Z3D::ZPointCloud* pointCloud READ pointCloud NOTIFY pointCloudChanged)
    Q_PROPERTY(QString name READ name CONSTANT)
    Q_PROPERTY(bool visible READ visible WRITE setVisible NOTIFY visibleChanged)
    Q_PROPERTY(bool highlighted READ highlighted WRITE setHighlighted NOTIFY highlightedChanged)
    Q_PROPERTY(QMatrix4x4 transformation READ transformation WRITE setTransformation NOTIFY transformationChanged)

public:
    explicit ZPointCloudListItem(const Z3D::ZPointCloudPtr &pointCloud,
                                 const QString &name,
                                 QObject *parent = nullptr);
    ~ZPointCloudListItem() override;

    Z3D::ZPointCloud *pointCloud() const;
    QString name() const;
    bool visible() const;
    bool highlighted() const;
    QMatrix4x4 transformation() const;

signals:
    void pointCloudChanged(Z3D::ZPointCloudPtr pointCloud);
    void visibleChanged(bool visible);
    void highlightedChanged(bool highlighted);
    void transformationChanged(QMatrix4x4 transformation);

public slots:
    void setPointCloud(Z3D::ZPointCloudPtr pointCloud);
    void setVisible(bool visible);
    void setHighlighted(bool highlighted);
    void setTransformation(QMatrix4x4 transformation);

private:
    Z3D::ZPointCloudPtr m_pointCloud;
    const QString m_name;
    bool m_visible = true;
    bool m_highlighted = false;
    QMatrix4x4 m_transformation; // default constructor == identity matrix
};

} // namespace Z3D
