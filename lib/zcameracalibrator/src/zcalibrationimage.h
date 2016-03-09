#pragma once

#include "zcameracalibrator_global.h"

#include <QMetaType> // for Q_DECLARE_METATYPE, to use in QVariant
#include <QObject>
#include <QPixmap>
#include <QPointer>

#include <opencv2/core/core.hpp>

namespace Z3D
{

/// forward declarations
class ZCalibrationPatternFinder;
class ZCameraCalibration;


class Z3D_CAMERACALIBRATOR_SHARED_EXPORT ZCalibrationImage : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString fileName  READ fileName  CONSTANT)
    Q_PROPERTY(int width         READ width     CONSTANT)
    Q_PROPERTY(int height        READ height    CONSTANT)
    Q_PROPERTY(ImageState state  READ state     NOTIFY stateChanged)

    Q_ENUMS(ImageState)
    Q_ENUMS(ImageViewStyle)

public:
    typedef QSharedPointer<ZCalibrationImage> Ptr;
    typedef QPointer<ZCalibrationImage> WeakPtr;

    enum ImageState {
        UnknownState      = 0x99,
        InvalidState      = 0x01,
        UncheckedState    = 0x02,
        CheckingState     = 0x04,
        CheckedState      = 0x08,
        PatternFoundState = 0x10 | CheckedState
    };

    enum ImageViewStyle {
        OriginalImage  = 0x00,
        ThumbnailImage = 0x01,
        PatternImage   = 0x02
    };

    explicit ZCalibrationImage(QString fileName, QObject *parent = 0);

    ~ZCalibrationImage();

    bool isValid();
    QString fileName() const;
    int width() const;
    int height() const;

    /// image
    QImage imageFromUrl(int viewStyle) const;
    QPixmap pixmapFromUrl(int viewStyle) const;

    /// image thumbnail
    QImage thumbnail() const;

    /// OpenCV mat
    cv::Mat mat() const;

    /// Detected pattern points
    std::vector<cv::Point2f> detectedPoints() const;
    std::vector<cv::Point3f> detectedPointsInRealCoordinates() const;

    cv::Matx33d rotation() const;
    cv::Point3d translation() const;


    Z3D::ZCalibrationImage::ImageState state() const;

signals:
    void stateChanged(Z3D::ZCalibrationImage::ImageState arg);

public slots:
    virtual bool findPattern(ZCalibrationPatternFinder *patternFinder);
    virtual bool estimateCalibrationPatternPosition(ZCameraCalibration *cameraCalibration);

protected slots:
    bool loadBasicImageInfo();

    void setState(Z3D::ZCalibrationImage::ImageState arg);

protected:
    QString m_fileName;

    QImage m_thumbnail;
    QImage m_patternThumbnail;

    int m_width;
    int m_height;

    std::vector<cv::Point2f> m_corners;
    std::vector<cv::Point3f> m_worldCorners;

    cv::Matx33d m_rotation;
    cv::Point3d m_translation;

    Z3D::ZCalibrationImage::ImageState m_state;

    QString m_patternFinderHash;
};

} // namespace Z3D

Q_DECLARE_METATYPE(Z3D::ZCalibrationImage::Ptr) /// to use in QVariant
