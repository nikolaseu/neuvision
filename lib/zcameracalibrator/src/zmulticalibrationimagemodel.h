#ifndef Z3D_CAMERACALIBRATOR___ZMULTICALIBRATIONIMAGEMODEL_H
#define Z3D_CAMERACALIBRATOR___ZMULTICALIBRATIONIMAGEMODEL_H

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

#endif // Z3D_CAMERACALIBRATOR___ZMULTICALIBRATIONIMAGEMODEL_H
