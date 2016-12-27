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

#include "zcameralistmodel.h"

namespace Z3D
{

ZCameraListModel::ZCameraListModel(QObject *parent) :
    QAbstractListModel(parent)
{

}


QHash<int, QByteArray> ZCameraListModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[CameraPtrRole] = "camera";
    return roles;
}


int ZCameraListModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return 0;
    else
        return m_cameraList.size();
}


QVariant ZCameraListModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || role != CameraPtrRole)
        return QVariant();

    ZCameraInterface* camera = m_cameraList.value(index.row());

    return QVariant::fromValue((QObject*) camera);
}


void ZCameraListModel::add(ZCameraInterface* camera)
{
    /// insert at the end
    int row = m_cameraList.size();
    beginInsertRows(QModelIndex(), row, row);

    //! add to list
    m_cameraList.insert(row, camera);

    endInsertRows();

    /// to remove from model when camera is deleted
    QObject::connect(camera, SIGNAL(destroyed(QObject*)),
                     this, SLOT(onCameraDeleted(QObject*)));
}

void ZCameraListModel::onCameraDeleted(QObject *object)
{
    ZCameraInterface *camera = static_cast<ZCameraInterface*>(object);

    int index = m_cameraList.indexOf(camera);
    beginRemoveRows(QModelIndex(), index, index);
    m_cameraList.removeAll(camera);
    endRemoveRows();
}

} // namespace Z3D
