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
