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

#include "zmulticalibrationimage.h"

#include <QAbstractListModel>
#include <QFutureWatcher>
#include <QList>

namespace Z3D
{

class ZMultiCalibrationImageModel : public QAbstractListModel
{
    Q_OBJECT

public:
    typedef QPointer<ZMultiCalibrationImageModel> WeakPtr;

    enum ZMultiCalibrationImageModelRoles {
        DataRole = Qt::UserRole + 1,
        FilenameRole
    };

    explicit ZMultiCalibrationImageModel(QObject *parent = 0);
    ~ZMultiCalibrationImageModel();

    QHash<int, QByteArray> roleNames() const;

    int rowCount(const QModelIndex &parent = QModelIndex()) const;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;

    const QList<Z3D::ZCalibrationImage::Ptr> &images();

signals:
    void newImagesAdded();

public slots:
    void clear();

    void addFolder(QString folder);

    void addImage(Z3D::ZMultiCalibrationImage::Ptr image);
    void addImageThreadSafe(Z3D::ZMultiCalibrationImage::Ptr image);
    void addImages(const QVector<Z3D::ZMultiCalibrationImage::Ptr > &images);

    void addImpl(Z3D::ZMultiCalibrationImage::Ptr image);

    Z3D::ZMultiCalibrationImage::Ptr imageAt(int index) const;

protected slots:
    void onImageStateChanged();

private:
    QList<Z3D::ZMultiCalibrationImage::Ptr> m_images;
    QList<Z3D::ZCalibrationImage::Ptr> m_allImagesList;

    QVector<Z3D::ZMultiCalibrationImage::Ptr> m_imagesToLoad;
    QFutureWatcher<void> m_futureWatcher;

    QMutex m_mutex;
};

} // namespace Z3D
