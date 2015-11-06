#ifndef Z3D_LASERTRIANGULATIONCALIBRATOR___ZLTSCALIBRATIONIMAGE_H
#define Z3D_LASERTRIANGULATIONCALIBRATOR___ZLTSCALIBRATIONIMAGE_H

#include "zcalibrationimage.h"

#include <QMetaType> // for Q_DECLARE_METATYPE, to use in QVariant

namespace Z3D
{

class ZLTSCalibrationImage : public ZCalibrationImage
{
    Q_OBJECT

public:
    typedef QSharedPointer<ZLTSCalibrationImage> Ptr;
    typedef QPointer<ZLTSCalibrationImage> WeakPtr;

    explicit ZLTSCalibrationImage(QString fileName, QObject *parent = 0);
    ~ZLTSCalibrationImage();

    virtual bool isValid();

    virtual QImage imageFromUrl(int viewStyle) const;
    virtual QPixmap pixmapFromUrl(int viewStyle) const;

    const std::vector<cv::Point2f> &validLaserPoints() const { return m_validLaserPoints; }

signals:

public slots:
    virtual bool findPattern(ZCalibrationPatternFinder *patternFinder);

protected:
    std::vector<cv::Point2f> m_laserPoints;

    std::vector<cv::Point2f> m_convexHullPoints;
    std::vector<cv::Point2f> m_validLaserPoints;
};

} // namespace Z3D

Q_DECLARE_METATYPE(Z3D::ZLTSCalibrationImage::Ptr) /// to use in QVariant

#endif // Z3D_LASERTRIANGULATIONCALIBRATOR___ZLTSCALIBRATIONIMAGE_H
