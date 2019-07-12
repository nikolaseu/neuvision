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

#include "zpointcloudlistmodel.h"

#include "zpointcloudlistitem.h"

#include "QQmlObjectListModel.h"

#include <QLoggingCategory>

namespace Z3D
{

namespace // anonymous namespace
{
Q_LOGGING_CATEGORY(loggingCategory, "z3d.zpointcloud.zpointcloudlistmodel")//, QtInfoMsg)
}

class ZPointCloudListModelPrivate : public QQmlObjectListModel<ZPointCloudListItem>
{
    Q_OBJECT

public:
    friend class ZPointCloudListModel;

    explicit ZPointCloudListModelPrivate(QObject *parent)
        : QQmlObjectListModel<ZPointCloudListItem>(parent)
    {

    }
};

ZPointCloudListModel::ZPointCloudListModel(QObject *parent)
    : QAbstractListModel(parent)
    , m_p(new ZPointCloudListModelPrivate(this))
{
    qDebug(loggingCategory) << "creating" << this;

    // we have to forward all signals :(

    //! These begin* signals are not available
    //    QObject::connect(m_p, &ZPointCloudListModelPrivate::beginMoveRows,
    //                     this, &ZPointCloudListModel::beginMoveRows);
    //    QObject::connect(m_p, &ZPointCloudListModelPrivate::beginInsertRows,
    //                     this, &ZPointCloudListModel::beginInsertRows);
    //    QObject::connect(m_p, &ZPointCloudListModelPrivate::beginRemoveRows,
    //                     this, &ZPointCloudListModel::beginRemoveRows);
    //    QObject::connect(m_p, &ZPointCloudListModelPrivate::beginResetModel,
    //                     this, &ZPointCloudListModel::beginResetModel);
    //    QObject::connect(m_p, &ZPointCloudListModelPrivate::beginMoveColumns,
    //                     this, &ZPointCloudListModel::beginMoveColumns);
    //    QObject::connect(m_p, &ZPointCloudListModelPrivate::beginInsertColumns,
    //                     this, &ZPointCloudListModel::beginInsertColumns);
    //    QObject::connect(m_p, &ZPointCloudListModelPrivate::beginRemoveColumns,
    //                     this, &ZPointCloudListModel::beginRemoveColumns);

    QObject::connect(m_p, &ZPointCloudListModelPrivate::rowsAboutToBeMoved,
                     this, &ZPointCloudListModel::rowsAboutToBeMoved);
    QObject::connect(m_p, &ZPointCloudListModelPrivate::rowsAboutToBeRemoved,
                     this, &ZPointCloudListModel::rowsAboutToBeRemoved);
    QObject::connect(m_p, &ZPointCloudListModelPrivate::rowsAboutToBeInserted,
                     this, &ZPointCloudListModel::rowsAboutToBeInserted);
    QObject::connect(m_p, &ZPointCloudListModelPrivate::rowsMoved,
                     this, &ZPointCloudListModel::rowsMoved);
    QObject::connect(m_p, &ZPointCloudListModelPrivate::rowsRemoved,
                     this, &ZPointCloudListModel::rowsRemoved);
    QObject::connect(m_p, &ZPointCloudListModelPrivate::rowsInserted,
                     this, &ZPointCloudListModel::rowsInserted);

    QObject::connect(m_p, &ZPointCloudListModelPrivate::dataChanged,
                     this, &ZPointCloudListModel::dataChanged);
}

ZPointCloudListModel::~ZPointCloudListModel()
{
    qDebug(loggingCategory) << "destroying" << this;
}

void ZPointCloudListModel::addPointCloud(ZPointCloudListItem *pointCloudItem)
{
    addPointClouds({ pointCloudItem });
}

void ZPointCloudListModel::addPointClouds(const std::vector<ZPointCloudListItem *> &pointCloudItems)
{
    for (auto *pointCloudItem : pointCloudItems) {
        /// make sure object lives in same thread
        pointCloudItem->moveToThread(thread());
    }

    /// just because I dont want to modify impl of qqmlobjectlistmodel
    QList<ZPointCloudListItem *> list;
    list.reserve(int(pointCloudItems.size()));
    std::copy(pointCloudItems.begin(), pointCloudItems.end(), std::back_inserter(list));

    QMetaObject::invokeMethod(m_p, [=](){
        m_p->append(list);
    });
}

int ZPointCloudListModel::rowCount(const QModelIndex &parent) const
{
    return m_p->rowCount(parent);
}

QVariant ZPointCloudListModel::data(const QModelIndex &index, int role) const
{
    return m_p->data(index, role);
}

QHash<int, QByteArray> ZPointCloudListModel::roleNames() const
{
    return m_p->roleNames();
}

} // namespace Z3D

// we have a Q_OBJECT in the cpp, so we have to force this
#include "zpointcloudlistmodel.moc"
