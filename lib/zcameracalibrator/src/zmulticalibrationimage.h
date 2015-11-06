#ifndef Z3D_CAMERACALIBRATOR___ZMULTICALIBRATIONIMAGE_H
#define Z3D_CAMERACALIBRATOR___ZMULTICALIBRATIONIMAGE_H

#include "zcameracalibrator_global.h"

#include "zcalibrationimage.h"

#include <QObject>

namespace Z3D
{

class ZMultiCalibrationImage : public QObject
{
    Q_OBJECT

public:
    typedef QSharedPointer<ZMultiCalibrationImage> Ptr;

    //explicit ZMultiCalibrationImage(QObject *parent = 0);
    ZMultiCalibrationImage(QList<Z3D::ZCalibrationImage::Ptr> images);
    ~ZMultiCalibrationImage();

    bool isValid();
    QString fileName() const;

    /// image thumbnail
    QImage thumbnail() const;

    const QList<Z3D::ZCalibrationImage::Ptr> &images() const;

    Z3D::ZCalibrationImage::Ptr image(int index) const;

    Z3D::ZCalibrationImage::ImageState state() const;

signals:
    void stateChanged(Z3D::ZCalibrationImage::ImageState arg);

public slots:

protected slots:
    void updateState();

protected:
    void setState(Z3D::ZCalibrationImage::ImageState arg);

    QList<Z3D::ZCalibrationImage::Ptr> m_images;

    Z3D::ZCalibrationImage::ImageState m_state;
};

} // namespace Z3D

Q_DECLARE_METATYPE(Z3D::ZMultiCalibrationImage::Ptr) /// to use in QVariant

#endif // Z3D_CAMERACALIBRATOR___ZMULTICALIBRATIONIMAGE_H
