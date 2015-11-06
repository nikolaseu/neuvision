#ifndef Z3D_CAMERACALIBRATOR___ZCHESSBOARDCALIBRATIONPATTERNFINDER_H
#define Z3D_CAMERACALIBRATOR___ZCHESSBOARDCALIBRATIONPATTERNFINDER_H

#include "zcalibrationpatternfinder.h"

namespace Z3D
{

class ZChessboardCalibrationPatternFinder : public ZCalibrationPatternFinder
{
    Q_OBJECT

    /// find chessboard corners options
    Q_PROPERTY(bool normalizeImage
               READ normalizeImage
               WRITE setNormalizeImage
               NOTIFY normalizeImageChanged)
    Q_PROPERTY(bool useAdaptiveThreshold
               READ useAdaptiveThreshold
               WRITE setUseAdaptiveThreshold
               NOTIFY useAdaptiveThresholdChanged)
    Q_PROPERTY(bool useFastCheck
               READ useFastCheck
               WRITE setUseFastCheck
               NOTIFY useFastCheckChanged)
    Q_PROPERTY(bool filterQuads
               READ filterQuads
               WRITE setFilterQuads
               NOTIFY filterQuadsChanged)

    /// corner sub pix options
    Q_PROPERTY(int subPixWinWidth
               READ subPixWinWidth
               WRITE setSubPixWinWidth
               NOTIFY subPixWinWidthChanged)
    Q_PROPERTY(int subPixWinHeight
               READ subPixWinHeight
               WRITE setSubPixWinHeight
               NOTIFY subPixWinHeightChanged)
    Q_PROPERTY(int subPixZeroZoneWinWidth
               READ subPixZeroZoneWinWidth
               WRITE setSubPixZeroZoneWinWidth
               NOTIFY subPixZeroZoneWinWidthChanged)
    Q_PROPERTY(int subPixZeroZoneWinHeight
               READ subPixZeroZoneWinHeight
               WRITE setSubPixZeroZoneWinHeight
               NOTIFY subPixZeroZoneWinHeightChanged)
    Q_PROPERTY(int subPixMaxIter
               READ subPixMaxIter
               WRITE setSubPixMaxIter
               NOTIFY subPixMaxIterChanged)
    Q_PROPERTY(double subPixEpsilon
               READ subPixEpsilon
               WRITE setSubPixEpsilon
               NOTIFY subPixEpsilonChanged)

public:
    explicit ZChessboardCalibrationPatternFinder(QObject *parent = 0);

    virtual QString name();

    virtual QWidget *configWidget();

    bool normalizeImage() const;
    bool useAdaptiveThreshold() const;
    bool useFastCheck() const;
    bool filterQuads() const;

    int subPixWinWidth() const;
    int subPixWinHeight() const;
    int subPixZeroZoneWinWidth() const;
    int subPixZeroZoneWinHeight() const;
    int subPixMaxIter() const;
    double subPixEpsilon() const;

signals:
    void normalizeImageChanged(bool arg);
    void useAdaptiveThresholdChanged(bool arg);
    void useFastCheckChanged(bool arg);
    void filterQuadsChanged(bool arg);

    void subPixWinWidthChanged(int arg);
    void subPixWinHeightChanged(int arg);
    void subPixZeroZoneWinWidthChanged(int arg);
    void subPixZeroZoneWinHeightChanged(int arg);
    void subPixMaxIterChanged(int arg);
    void subPixEpsilonChanged(double arg);

public slots:
    void setNormalizeImage(bool normalizeImage);
    void setUseAdaptiveThreshold(bool useAdaptiveThreshold);
    void setUseFastCheck(bool useFastCheck);
    void setFilterQuads(bool filterQuads);

    void setSubPixWinWidth(int arg);
    void setSubPixWinHeight(int arg);
    void setSubPixZeroZoneWinWidth(int arg);
    void setSubPixZeroZoneWinHeight(int arg);
    void setSubPixMaxIter(int arg);
    void setSubPixEpsilon(double arg);

    virtual bool findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &objectPoints);

protected slots:
    virtual void updateConfigHash();

protected:
    /// findChessboardCorners flags
    int m_findChessboardFlags;

    /// cornerSubPix config
    int m_subPixWinWidth;
    int m_subPixWinHeight;
    int m_subPixZeroZoneWinWidth;
    int m_subPixZeroZoneWinHeight;
    int m_subPixMaxIter;
    double m_subPixEpsilon;

    QWidget *m_configWidget;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZCHESSBOARDCALIBRATIONPATTERNFINDER_H
