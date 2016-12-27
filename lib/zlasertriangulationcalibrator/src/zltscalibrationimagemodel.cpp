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

#include "zltscalibrationimagemodel.h"

#include <QDebug>
#include <QDir>
#if QT_VERSION < 0x050000
#include <QtConcurrentMap>
//#include <QtConcurrentRun>
#else
#include <QtConcurrent>
#endif

#include <QUrl>

namespace Z3D
{

struct ParallelImageCheckImpl
{
    ParallelImageCheckImpl(ZLTSCalibrationImageModel *imageModel)
        : m_imageModel(imageModel) { }

    void operator()(const Z3D::ZLTSCalibrationImage::Ptr &image)
    {
        if (image->isValid()) {
            m_imageModel->add(image);
        }
    }

    ZLTSCalibrationImageModel *m_imageModel;
};



ZLTSCalibrationImageModel::ZLTSCalibrationImageModel(QObject *parent) :
    QAbstractListModel(parent),
    m_width(0),
    m_height(0)
{
    /// to notify when the loading of images has finished
    QObject::connect(&m_futureWatcher, SIGNAL(finished()),
                     this, SIGNAL(newImagesAdded()));
}

QHash<int, QByteArray> ZLTSCalibrationImageModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[DataRole] = "calibrationImage";
    return roles;
}

int ZLTSCalibrationImageModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return 0;
    else
        return m_images.size();
}

QVariant ZLTSCalibrationImageModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() < 0 || index.row() >= m_images.size())
        return QVariant();

    Z3D::ZLTSCalibrationImage::Ptr imageObject = m_images.value(index.row());

    switch (role) {
    case DataRole:
        return QVariant::fromValue(imageObject);
    case Qt::ToolTipRole:
    case FilenameRole:
        return imageObject->fileName();
    case Qt::DecorationRole:
        return imageObject->pixmapFromUrl(Z3D::ZLTSCalibrationImage::ThumbnailImage);
    }

    return QVariant();
}

const QList<Z3D::ZLTSCalibrationImage::Ptr > &ZLTSCalibrationImageModel::images() const
{
    return m_images;
}

void ZLTSCalibrationImageModel::addFolder(QString folder)
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

    //QDir imageDir(QUrl(folder).toLocalFile());
    QDir imageDir(folder);

    QStringList imageFiles = imageDir.entryList(filters, QDir::Files, QDir::Name);

    /// this is declared static because we need to keep it in memory (and I don't
    /// want to use heap allocation to avoid memory leaks)
    static QVector< Z3D::ZLTSCalibrationImage::Ptr > images;

    /// this is the part that we need to avoid running when the concurrent
    /// loading still hasn't finished
    images.clear();
    images.reserve(imageFiles.size());

    /// images must be created from this thread, this is required by the Qml engine
    foreach (QString file, imageFiles) {
        Z3D::ZLTSCalibrationImage::Ptr image( new ZLTSCalibrationImage(imageDir.absoluteFilePath(file)) );
        images.push_back(image);
    }

    /// add images to model
    addImages(images);
}

void ZLTSCalibrationImageModel::add(Z3D::ZLTSCalibrationImage::Ptr image)
{
    /// executes addImpl from the object thread, not from the thread where this
    /// function is called. this is needed for the view to update correctly when
    /// we add images in parallel (like in addFolder, using QtConcurrent::map)
    QMetaObject::invokeMethod(this, "addImpl", Qt::AutoConnection,
                              Q_ARG(Z3D::ZLTSCalibrationImage::Ptr, image));
}

void ZLTSCalibrationImageModel::addImages(const QVector<Z3D::ZLTSCalibrationImage::Ptr > &images)
{
    /// parallelize the checking of image validity before adding to model
    m_futureWatcher.setFuture( QtConcurrent::map(images, ParallelImageCheckImpl(this)) );

    /* VERSION no paralela
    foreach (Z3D::ZCalibrationImage::Ptr image, images) {
        if (image->isValid()) {
            add(image);
        }
    }*/
}

void ZLTSCalibrationImageModel::addImpl(Z3D::ZLTSCalibrationImage::Ptr image)
{
    /// to avoid entering this section from more than one thread at a time
    QMutexLocker mutexLocker(&m_mutex);

    int row = m_images.size();

    if (row < 1) {
        m_width = image->width();
        m_height = image->height();
        emit imageSizeChanged();
        qDebug() << "image size:" << m_width << "x" << m_height;
    }

    if (image->width() != m_width || image->height() != m_height) {
        qWarning() << "image not added. it is not the same size:" << image->fileName();
        return;
    }

    beginInsertRows(QModelIndex(), row, row);

    //! add to list
    m_images.insert(row, image);

    endInsertRows();

    //! connect to image signals
    QObject::connect(image.data(), SIGNAL(stateChanged(Z3D::ZCalibrationImage::ImageState)),
                     this, SLOT(onImageStateChanged()));
}

Z3D::ZLTSCalibrationImage::Ptr ZLTSCalibrationImageModel::imageAt(int index)
{
    if (index >= 0 && index < m_images.size()) {
        return m_images[index];
    } else {
        qCritical() << "invalid image index requested:" << index << " - model size:" << m_images.size();
        return Z3D::ZLTSCalibrationImage::Ptr(0);
    }
}

void ZLTSCalibrationImageModel::onImageStateChanged()
{
    Z3D::ZLTSCalibrationImage *changedImage = qobject_cast<Z3D::ZLTSCalibrationImage*>(sender());
    for (int i=0; i<m_images.size(); ++i) {
        if (m_images[i].data() == changedImage) {
            emit dataChanged(index(i), index(i));
            break;
        }
    }
}

} // namespace Z3D
