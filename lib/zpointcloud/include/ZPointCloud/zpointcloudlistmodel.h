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

#include "ZPointCloud/zpointcloud_global.h"

#include <QAbstractListModel>

namespace Z3D
{
class ZPointCloudListItem;
class ZPointCloudListModelPrivate;

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZPointCloudListModel : public QAbstractListModel
{
    Q_OBJECT

public:
    enum Roles {
        ZPointCloudListItemRole = Qt::UserRole + 999
    };

    explicit ZPointCloudListModel(QObject *parent = nullptr);
    ~ZPointCloudListModel() override;

    void clear();

    // takes ownership of the item
    void addPointCloud(ZPointCloudListItem *pointCloudItem);
    void addPointClouds(const std::vector<ZPointCloudListItem *> &pointCloudItems);

    // QAbstractItemModel interface
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex &index, const QVariant &value, int role) override;
    QHash<int, QByteArray> roleNames() const override;

private:
    ZPointCloudListModelPrivate *m_p;
};

} // namespace Z3D
