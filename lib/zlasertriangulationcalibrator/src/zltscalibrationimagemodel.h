#pragma once

#include "zltscalibrationimage.h"

#include <QAbstractListModel>
#include <QFutureWatcher>
#include <QList>
#include <QSharedPointer>

namespace Z3D
{

class ZLTSCalibrationImageModel : public QAbstractListModel
{
    Q_OBJECT

    Q_PROPERTY(int imageWidth  READ imageWidth  NOTIFY imageSizeChanged)
    Q_PROPERTY(int imageHeight READ imageHeight NOTIFY imageSizeChanged)

public:
    typedef QPointer<ZLTSCalibrationImageModel> WeakPtr;

    enum ZLTSCalibrationImageModelRoles {
        DataRole = Qt::UserRole + 1,
        FilenameRole
    };

    explicit ZLTSCalibrationImageModel(QObject *parent = 0);

    QHash<int, QByteArray> roleNames() const;

    int rowCount(const QModelIndex &parent = QModelIndex()) const;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;

    int imageWidth() { return m_width; }
    int imageHeight() { return m_height; }

    const QList< Z3D::ZLTSCalibrationImage::Ptr > &images() const;

signals:
    void imageSizeChanged();
    void newImagesAdded();

public slots:
    void addFolder(QString folder);

    void add(Z3D::ZLTSCalibrationImage::Ptr image);
    void addImages(const QVector<Z3D::ZLTSCalibrationImage::Ptr > &images);

    void addImpl(Z3D::ZLTSCalibrationImage::Ptr image);

    Z3D::ZLTSCalibrationImage::Ptr imageAt(int index);

protected slots:
    void onImageStateChanged();

private:
    QList< Z3D::ZLTSCalibrationImage::Ptr > m_images;

    int m_width;
    int m_height;

    QFutureWatcher<void> m_futureWatcher;

    QMutex m_mutex;
};

} // namespace Z3D
