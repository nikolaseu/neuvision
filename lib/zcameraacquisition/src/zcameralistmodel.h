#ifndef Z3D_CAMERAACQUISITION___ZCAMERALISTMODEL_H
#define Z3D_CAMERAACQUISITION___ZCAMERALISTMODEL_H

#include "zcameraacquisition_global.h"
#include "zcamerainterface.h"

#include <QAbstractListModel>
#include <QSharedPointer>

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraListModel : public QAbstractListModel
{
    Q_OBJECT

public:
    enum CameraListModelRoles {
        CameraPtrRole = Qt::UserRole + 1
    };

    explicit ZCameraListModel(QObject *parent = 0);

    QHash<int, QByteArray> roleNames() const;

    int rowCount(const QModelIndex &parent) const;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;

signals:

public slots:
    void add(ZCameraInterface *camera);

protected slots:
    void onCameraDeleted(QObject *object);

protected:
    QList<Z3D::ZCameraInterface*> m_cameraList;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION___ZCAMERALISTMODEL_H
