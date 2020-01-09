//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2019 Nicolas Ulrich <nikolaseu@gmail.com>
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

#include "ZPointCloud/zpointcloudlistitem.h"

#include <QLoggingCategory>

namespace Z3D
{

namespace // anonymous namespace
{
Q_LOGGING_CATEGORY(loggingCategory, "z3d.zpointcloud.zpointcloudlistitem", QtInfoMsg)
}

ZPointCloudListItem::ZPointCloudListItem(const ZPointCloudPtr &pointCloud,
                                         const QString &name,
                                         QObject *parent)
    : QObject(parent)
    , m_pointCloud(pointCloud)
    , m_name(name)
{
    qDebug(loggingCategory) << "creating" << this;
}

ZPointCloudListItem::~ZPointCloudListItem()
{
    qDebug(loggingCategory) << "destroying" << this;
}

ZPointCloud *ZPointCloudListItem::pointCloud() const
{
    return m_pointCloud.get();
}

QString ZPointCloudListItem::name() const
{
    return m_name;
}

bool ZPointCloudListItem::visible() const
{
    return m_visible;
}

bool ZPointCloudListItem::highlighted() const
{
    return m_highlighted;
}

QMatrix4x4 ZPointCloudListItem::transformation() const
{
    return m_transformation;
}

void ZPointCloudListItem::setVisible(bool visible)
{
    if (m_visible == visible) {
        return;
    }

    m_visible = visible;
    emit visibleChanged(m_visible);
}

void ZPointCloudListItem::setHighlighted(bool highlighted)
{
    if (m_highlighted == highlighted) {
        return;
    }

    m_highlighted = highlighted;
    emit highlightedChanged(m_highlighted);
}

void ZPointCloudListItem::setTransformation(QMatrix4x4 transformation)
{
    if (m_transformation == transformation) {
        return;
    }

    m_transformation = transformation;
    emit transformationChanged(m_transformation);
}

} // namespace Z3D
