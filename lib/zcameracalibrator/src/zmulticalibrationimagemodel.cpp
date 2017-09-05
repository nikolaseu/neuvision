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

#include "zmulticalibrationimagemodel.h"

#include <QDir>
#include <QtConcurrent>

namespace Z3D
{

ZMultiCalibrationImageModel::ZMultiCalibrationImageModel(QObject *parent)
    : QAbstractListModel(parent)
{
    /// to notify when the loading of images has finished
    QObject::connect(&m_futureWatcher, SIGNAL(finished()),
                     this, SIGNAL(newImagesAdded()));
}

ZMultiCalibrationImageModel::~ZMultiCalibrationImageModel()
{

}

QHash<int, QByteArray> ZMultiCalibrationImageModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[DataRole] = "multiCalibrationImage";
    return roles;
}

int ZMultiCalibrationImageModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return 0;
    else
        return m_images.size();
}

QVariant ZMultiCalibrationImageModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() < 0 || index.row() >= m_images.size())
        return QVariant();

    Z3D::ZMultiCalibrationImage::Ptr imageObject = imageAt(index.row());

    switch (role) {
    case DataRole:
        return QVariant::fromValue(imageObject);
    case Qt::ToolTipRole:
    case FilenameRole:
        return imageObject->fileName();
    case Qt::DecorationRole:
        return imageObject->thumbnail();
    }

    return QVariant();
}

const QList<ZCalibrationImage::Ptr> &ZMultiCalibrationImageModel::images()
{
    m_allImagesList.clear();
    foreach (ZMultiCalibrationImage::Ptr multiImage, m_images) {
        m_allImagesList << multiImage->images();
    }
    return m_allImagesList;
}

void ZMultiCalibrationImageModel::clear()
{
    beginResetModel();

    //! clear list
    m_images.clear();

    endResetModel();
}

void ZMultiCalibrationImageModel::addFolder(QString folder)
{
    /// we must wait for the previous task to finish
    /// because we use the static vector "images"
    if (m_futureWatcher.isRunning()) {
        qWarning() << "waiting for previous task to finish...";
        m_futureWatcher.waitForFinished();
    }

    qDebug() << "loading calibration images from" << folder;

    QStringList filters;
    filters << "*.jpg" << "*.bmp" << "*.png";

    QDir camerasDir(folder);
    QStringList cameraFolders = camerasDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);

    if (cameraFolders.empty()) {
        qWarning() << "not even one camera folder found!";
        return;
    }

    QString firstCameraFolder = cameraFolders.first();

    QDir imageDir(QString("%1/%2").arg(folder).arg(firstCameraFolder));

    QStringList imageFiles = imageDir.entryList(filters, QDir::Files, QDir::Name);

    /// this is the part that we need to avoid running when the concurrent
    /// loading still hasn't finished
    m_imagesToLoad.clear();
    m_imagesToLoad.reserve(imageFiles.size());

    /// images must be created from this thread, this is required by the Qml engine
    foreach (QString file, imageFiles) {
        QList<Z3D::ZCalibrationImage::Ptr> imagesList;
        foreach (QString cameraFolder, cameraFolders) {
            Z3D::ZCalibrationImage::Ptr image( new ZCalibrationImage(camerasDir.absoluteFilePath(QString("%1/%2").arg(cameraFolder).arg(file))) );
            imagesList << image;
        }

        Z3D::ZMultiCalibrationImage::Ptr images(new Z3D::ZMultiCalibrationImage(imagesList));
        m_imagesToLoad.push_back(images);
    }

    /// add images to model
    addImages(m_imagesToLoad);
}

void ZMultiCalibrationImageModel::addImage(ZMultiCalibrationImage::Ptr image)
{
    /// we must wait for the previous task to finish
    /// because we use the static vector "images"
    if (m_futureWatcher.isRunning()) {
        qWarning() << "waiting for previous task to finish...";
        m_futureWatcher.waitForFinished();
    }

    m_imagesToLoad.clear();
    m_imagesToLoad.push_back(image);

    /// add images to model
    addImages(m_imagesToLoad);
}

void ZMultiCalibrationImageModel::addImageThreadSafe(ZMultiCalibrationImage::Ptr image)
{
    /// executes addImpl from the object thread, not from the thread where this
    /// function is called. this is needed for the view to update correctly when
    /// we add images in parallel (like in addImages, using QtConcurrent::map)
    QMetaObject::invokeMethod(this, "addImpl", Qt::AutoConnection,
                              Q_ARG(Z3D::ZMultiCalibrationImage::Ptr, image));
}

void ZMultiCalibrationImageModel::addImages(const QVector<ZMultiCalibrationImage::Ptr> &images)
{
    /// parallelize the checking of image validity before adding to model
    m_futureWatcher.setFuture(QtConcurrent::map(images, [=](const auto &image) {
        if (image->isValid()) {
            addImageThreadSafe(image);
        }
    }));
}

void ZMultiCalibrationImageModel::addImpl(ZMultiCalibrationImage::Ptr image)
{
    /// to avoid entering this section from more than one thread at a time
    QMutexLocker mutexLocker(&m_mutex);

    int row = m_images.size();

    /*if (row < 1) {
        m_width = image->width();
        m_height = image->height();
        emit imageSizeChanged();
        qDebug() << "image size:" << m_width << "x" << m_height;
    }

    if (image->width() != m_width || image->height() != m_height) {
        qWarning() << "image not added. it is not the same size:" << image->fileName();
        return;
    }*/

    beginInsertRows(QModelIndex(), row, row);

    //! add to list
    m_images.insert(row, image);

    endInsertRows();

    //! connect to image signals
    QObject::connect(image.data(), SIGNAL(stateChanged(Z3D::ZCalibrationImage::ImageState)),
                     this, SLOT(onImageStateChanged()));
}

ZMultiCalibrationImage::Ptr ZMultiCalibrationImageModel::imageAt(int index) const
{
    if (index >= 0 && index < m_images.size()) {
        return m_images[index];
    } else {
        qCritical() << "invalid image index requested:" << index << " - model size:" << m_images.size();
        return Z3D::ZMultiCalibrationImage::Ptr(nullptr);
    }
}

void ZMultiCalibrationImageModel::onImageStateChanged()
{
    Z3D::ZMultiCalibrationImage *changedImage = qobject_cast<Z3D::ZMultiCalibrationImage*>(sender());
    for (int i=0; i<m_images.size(); ++i) {
        if (m_images[i].data() == changedImage) {
            emit dataChanged(index(i), index(i));
            break;
        }
    }
}

} // namespace Z3D
