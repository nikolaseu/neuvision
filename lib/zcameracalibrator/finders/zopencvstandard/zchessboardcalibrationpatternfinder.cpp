#include "zchessboardcalibrationpatternfinder.h"

#include "zchessboardcalibrationpatternfinderconfigwidget.h"

#include <opencv2/calib3d/calib3d.hpp> // findChessboardCorners
#include <opencv2/imgproc/imgproc.hpp> // cornerSubPix

namespace Z3D
{

ZChessboardCalibrationPatternFinder::ZChessboardCalibrationPatternFinder(QObject *parent)
    : ZCalibrationPatternFinder(parent)
    , m_findChessboardFlags(cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE)
    , m_subPixWinWidth(11)
    , m_subPixWinHeight(11)
    , m_subPixZeroZoneWinWidth(-1)
    , m_subPixZeroZoneWinHeight(-1)
    , m_subPixMaxIter(50)
    , m_subPixEpsilon(1e-5)
    , m_configWidget(0)
{
    QObject::connect(this, SIGNAL(useAdaptiveThresholdChanged(bool)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(useFastCheckChanged(bool)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(filterQuadsChanged(bool)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(normalizeImageChanged(bool)),
                     this, SLOT(updateConfigHash()));

    QObject::connect(this, SIGNAL(subPixWinWidthChanged(int)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(subPixWinHeightChanged(int)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(subPixZeroZoneWinWidthChanged(int)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(subPixZeroZoneWinHeightChanged(int)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(subPixMaxIterChanged(int)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(subPixEpsilonChanged(double)),
                     this, SLOT(updateConfigHash()));

    /// generate current config hash
    updateConfigHash();
}

QString ZChessboardCalibrationPatternFinder::name()
{
    return QLatin1String("Chessboard");
}

QWidget *ZChessboardCalibrationPatternFinder::configWidget()
{
    if (!m_configWidget) {
        m_configWidget = new ZChessboardCalibrationPatternFinderConfigWidget(this);
        m_configWidget->setVisible(false);
    }

    return m_configWidget;
}

bool ZChessboardCalibrationPatternFinder::normalizeImage() const
{
    return m_findChessboardFlags & cv::CALIB_CB_NORMALIZE_IMAGE;
}

bool ZChessboardCalibrationPatternFinder::useAdaptiveThreshold() const
{
    return m_findChessboardFlags & cv::CALIB_CB_ADAPTIVE_THRESH;
}

bool ZChessboardCalibrationPatternFinder::useFastCheck() const
{
    return m_findChessboardFlags & cv::CALIB_CB_FAST_CHECK;
}

bool ZChessboardCalibrationPatternFinder::filterQuads() const
{
    return m_findChessboardFlags & cv::CALIB_CB_FILTER_QUADS;
}

int ZChessboardCalibrationPatternFinder::subPixWinWidth() const
{
    return m_subPixWinWidth;
}

int ZChessboardCalibrationPatternFinder::subPixWinHeight() const
{
    return m_subPixWinHeight;
}

int ZChessboardCalibrationPatternFinder::subPixZeroZoneWinWidth() const
{
    return m_subPixZeroZoneWinWidth;
}

int ZChessboardCalibrationPatternFinder::subPixZeroZoneWinHeight() const
{
    return m_subPixZeroZoneWinHeight;
}

int ZChessboardCalibrationPatternFinder::subPixMaxIter() const
{
    return m_subPixMaxIter;
}

double ZChessboardCalibrationPatternFinder::subPixEpsilon() const
{
    return m_subPixEpsilon;
}

void ZChessboardCalibrationPatternFinder::setNormalizeImage(bool normalizeImage)
{
    if (this->normalizeImage() != normalizeImage) {
        if (normalizeImage)
            m_findChessboardFlags |=  cv::CALIB_CB_NORMALIZE_IMAGE;
        else
            m_findChessboardFlags &= ~cv::CALIB_CB_NORMALIZE_IMAGE;
        emit normalizeImageChanged(normalizeImage);
    }
}

void ZChessboardCalibrationPatternFinder::setUseAdaptiveThreshold(bool useAdaptiveThreshold)
{
    if (this->useAdaptiveThreshold() != useAdaptiveThreshold) {
        if (useAdaptiveThreshold)
            m_findChessboardFlags |=  cv::CALIB_CB_ADAPTIVE_THRESH;
        else
            m_findChessboardFlags &= ~cv::CALIB_CB_ADAPTIVE_THRESH;
        emit useAdaptiveThresholdChanged(useAdaptiveThreshold);
    }
}

void ZChessboardCalibrationPatternFinder::setUseFastCheck(bool useFastCheck)
{
    if (this->useFastCheck() != useFastCheck) {
        if (useFastCheck)
            m_findChessboardFlags |=  cv::CALIB_CB_FAST_CHECK;
        else
            m_findChessboardFlags &= ~cv::CALIB_CB_FAST_CHECK;
        emit useFastCheckChanged(useFastCheck);
    }
}

void ZChessboardCalibrationPatternFinder::setFilterQuads(bool filterQuads)
{
    if (this->filterQuads() != filterQuads) {
        if (filterQuads)
            m_findChessboardFlags |=  cv::CALIB_CB_FILTER_QUADS;
        else
            m_findChessboardFlags &= ~cv::CALIB_CB_FILTER_QUADS;
        emit filterQuadsChanged(filterQuads);
    }
}

void ZChessboardCalibrationPatternFinder::setSubPixWinWidth(int arg)
{
    if (m_subPixWinWidth != arg) {
        m_subPixWinWidth = arg;
        emit subPixWinWidthChanged(arg);
    }
}

void ZChessboardCalibrationPatternFinder::setSubPixWinHeight(int arg)
{
    if (m_subPixWinHeight != arg) {
        m_subPixWinHeight = arg;
        emit subPixWinHeightChanged(arg);
    }
}

void ZChessboardCalibrationPatternFinder::setSubPixZeroZoneWinWidth(int arg)
{
    if (m_subPixZeroZoneWinWidth != arg) {
        m_subPixZeroZoneWinWidth = arg;
        emit subPixZeroZoneWinWidthChanged(arg);
    }
}

void ZChessboardCalibrationPatternFinder::setSubPixZeroZoneWinHeight(int arg)
{
    if (m_subPixZeroZoneWinHeight != arg) {
        m_subPixZeroZoneWinHeight = arg;
        emit subPixZeroZoneWinHeightChanged(arg);
    }
}

void ZChessboardCalibrationPatternFinder::setSubPixMaxIter(int arg)
{
    if (m_subPixMaxIter != arg) {
        m_subPixMaxIter = arg;
        emit subPixMaxIterChanged(arg);
    }
}

void ZChessboardCalibrationPatternFinder::setSubPixEpsilon(double arg)
{
    if (m_subPixEpsilon != arg) {
        m_subPixEpsilon = arg;
        emit subPixEpsilonChanged(arg);
    }
}

bool ZChessboardCalibrationPatternFinder::findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &objectPoints)
{
    ZCalibrationPatternFinder::findCalibrationPattern(image, corners, objectPoints);

    bool found = cv::findChessboardCorners(image,
                                           m_boardSize,
                                           corners,
                                           m_findChessboardFlags);

    if (found) {
        cv::cornerSubPix(image,
                         corners,
                         cv::Size(m_subPixWinWidth, m_subPixWinHeight),
                         cv::Size(m_subPixZeroZoneWinWidth, m_subPixZeroZoneWinHeight),
                         cv::TermCriteria( cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, m_subPixMaxIter, m_subPixEpsilon ));

        /// clear vector
        objectPoints.clear();

        /// reserve space
        objectPoints.reserve(columns() * rows());

        /// generate world object coordinates
        for (int h=0; h<rows(); ++h) {
            for (int w=0; w<columns(); ++w) {
                objectPoints.push_back(cv::Point3f(m_colWidth * w, m_rowHeight * h, 0.f));
            }
        }
    }

    return found;
}

void ZChessboardCalibrationPatternFinder::updateConfigHash()
{
    /// generate unique "hash" for the current configuration
    QString hash = QString("%1|%2|%3|%4|%5|%6|%7|%8")
            .arg(getBaseHash())
            .arg(m_findChessboardFlags)
            .arg(m_subPixWinWidth)
            .arg(m_subPixWinHeight)
            .arg(m_subPixZeroZoneWinWidth)
            .arg(m_subPixZeroZoneWinHeight)
            .arg(m_subPixMaxIter)
            .arg(m_subPixEpsilon);

    if (m_configHash != hash) {
        m_configHash = hash;
        emit configHashChanged(m_configHash);
    }
}

} // namespace Z3D
