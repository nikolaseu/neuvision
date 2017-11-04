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

#include "zcameracalibrator_fwd.h"

#include <QAbstractListModel>
#include <QFutureWatcher>
#include <QList>

namespace Z3D
{

class ZCalibrationImageModel : public QAbstractListModel
{
    Q_OBJECT

    Q_PROPERTY(int imageWidth  READ imageWidth  NOTIFY imageSizeChanged)
    Q_PROPERTY(int imageHeight READ imageHeight NOTIFY imageSizeChanged)

public:
    enum ZCalibrationImageModelRoles {
        DataRole = Qt::UserRole + 1,
        FilenameRole
    };

    explicit ZCalibrationImageModel(QObject *parent = nullptr);

    QHash<int, QByteArray> roleNames() const;

    int rowCount(const QModelIndex &parent = QModelIndex()) const;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;

    int imageWidth() { return m_width; }
    int imageHeight() { return m_height; }

    const QList< Z3D::ZCalibrationImagePtr > &images() const;

signals:
    void imageSizeChanged();
    void newImagesAdded();

public slots:
    void clear();

    void addFolder(QString folder);

    void addImage(Z3D::ZCalibrationImagePtr image);
    void addImageThreadSafe(Z3D::ZCalibrationImagePtr image);
    void addImages(const QVector<Z3D::ZCalibrationImagePtr > &images);

    void addImpl(Z3D::ZCalibrationImagePtr image);

    Z3D::ZCalibrationImagePtr imageAt(int index) const;

protected slots:
    void onImageStateChanged();

private:
    QList< Z3D::ZCalibrationImagePtr > m_images;

    int m_width;
    int m_height;

    QVector< Z3D::ZCalibrationImagePtr > m_imagesToLoad;
    QFutureWatcher<void> m_futureWatcher;

    QMutex m_mutex;
};

} // namespace Z3D
