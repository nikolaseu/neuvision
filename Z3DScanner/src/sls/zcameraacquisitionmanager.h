#ifndef ZCAMERAACQUISITIONMANAGER_H
#define ZCAMERAACQUISITIONMANAGER_H

#include <QObject>

#include "zcameraimage.h"
#include "zcamerainterface.h"

namespace Z3D {

class ZCameraAcquisitionManager : public QObject
{
    Q_OBJECT

public:
    explicit ZCameraAcquisitionManager(std::vector<Z3D::ZCameraInterface::Ptr> cameras, QObject *parent = 0);

signals:
    void acquisitionReady(QString acquisitionId);
    void imagesAcquired(std::vector<Z3D::ZImageGrayscale::Ptr> images, QString id);
    void acquisitionFinished(std::vector< std::vector<Z3D::ZImageGrayscale::Ptr> > acquiredImages, QString acquisitionId);

public slots:
    void prepareAcquisition(QString acquisitionId);
    void acquireSingle(QString id);
    void finishAcquisition();

private:
    std::vector<Z3D::ZCameraInterface::Ptr> m_cameras;

    // 1st index: image number / order
    // 2nd index: camera index
    std::vector< std::vector<Z3D::ZImageGrayscale::Ptr> > m_images;

    QString m_acquisitionId;

    bool m_debugMode;
};

} // namespace Z3D

#endif // ZCAMERAACQUISITIONMANAGER_H
