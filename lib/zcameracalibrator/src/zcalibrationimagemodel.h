#pragma once

#include "zcalibrationimage.h"

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
    typedef QPointer<ZCalibrationImageModel> WeakPtr;

    enum ZCalibrationImageModelRoles {
        DataRole = Qt::UserRole + 1,
        FilenameRole
    };

    explicit ZCalibrationImageModel(QObject *parent = 0);

    QHash<int, QByteArray> roleNames() const;

    int rowCount(const QModelIndex &parent = QModelIndex()) const;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;

    int imageWidth() { return m_width; }
    int imageHeight() { return m_height; }

    const QList< Z3D::ZCalibrationImage::Ptr > &images() const;

signals:
    void imageSizeChanged();
    void newImagesAdded();

public slots:
    void clear();

    void addFolder(QString folder);

    void addImage(Z3D::ZCalibrationImage::Ptr image);
    void addImageThreadSafe(Z3D::ZCalibrationImage::Ptr image);
    void addImages(const QVector<Z3D::ZCalibrationImage::Ptr > &images);

    void addImpl(Z3D::ZCalibrationImage::Ptr image);

    Z3D::ZCalibrationImage::Ptr imageAt(int index) const;

protected slots:
    void onImageStateChanged();

private:
    QList< Z3D::ZCalibrationImage::Ptr > m_images;

    int m_width;
    int m_height;

    QVector< Z3D::ZCalibrationImage::Ptr > m_imagesToLoad;
    QFutureWatcher<void> m_futureWatcher;

    QMutex m_mutex;
};

} // namespace Z3D
