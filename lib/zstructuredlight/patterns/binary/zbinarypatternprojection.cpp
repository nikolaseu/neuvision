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

#include "zbinarypatternprojection.h"

#include "zbinarypatterndecoder.h"
#include "zcameraimage.h"
#include "zdecodedpattern.h"
#include "zprojectedpattern.h"

#include <opencv2/imgcodecs.hpp>

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QDebug>
#include <QThread>
#include <QQmlContext>
#include <QQuickView>


namespace Z3D
{

ZBinaryPatternProjection::ZBinaryPatternProjection(QObject *parent)
    : ZPatternProjection(parent)
    , m_dlpview(nullptr)
    , m_delayMs(400)
    , m_noiseThreshold(15)
    , m_automaticPatternCount(true)
    , m_numPatterns(11)
    , m_intensity(1.)
    , m_currentPattern(0)
    , m_inverted(false)
    , m_vertical(true)
    , m_useGrayBinary(true)
    , m_debugMode(false)
    , m_previewEnabled(false)
{
    m_dlpview = new QQuickView();
    m_dlpview->rootContext()->setContextProperty("config", this);
    m_dlpview->rootContext()->setContextProperty("window", m_dlpview);
    m_dlpview->setSource(QUrl("qrc:///zstructuredlight/qml/binarypatternprojection.qml"));
    m_dlpview->setResizeMode(QQuickView::SizeRootObjectToView);
}

ZBinaryPatternProjection::~ZBinaryPatternProjection()
{
    qDebug() << "deleting projector window...";
    if (m_dlpview) {
        m_dlpview->deleteLater();
    }
}

QString ZBinaryPatternProjection::id() const
{
    return QString(metaObject()->className());
}

QString ZBinaryPatternProjection::displayName() const
{
    return tr("Binary");
}

void ZBinaryPatternProjection::showProjectionWindow()
{
    m_dlpview->showNormal();
    m_dlpview->showFullScreen();
}

void ZBinaryPatternProjection::hideProjectionWindow()
{
    m_dlpview->hide();
}

void ZBinaryPatternProjection::setProjectionWindowGeometry(const QRect &geometry)
{
    m_dlpview->setGeometry(geometry);

    updateMaxUsefulPatterns();
}

void ZBinaryPatternProjection::beginScan()
{
    bool previewWasEnabled = setPreviewEnabled(true);

    /// to skip useless patterns
    int firstPatternToShow = 0;

    //! FIXME mejorar esto
    if (!m_automaticPatternCount) {
        firstPatternToShow = m_maxUsefulPatterns - m_numPatterns;
    }

    qDebug() << "maxUsefulPatterns:" << m_maxUsefulPatterns;
    qDebug() << "numPatterns:" << m_numPatterns << "firstPatternToShow:" << firstPatternToShow;

    QString scanTmpFolder = QString("tmp/dlpscans/%1").arg(QDateTime::currentDateTime().toString("yyyy.MM.dd_hh.mm.ss"));

    emit prepareAcquisition(scanTmpFolder);

    setVertical(m_vertical);

    /// acquisition time
    QTime acquisitionTime;
    acquisitionTime.start();

    for (int iPattern=0; iPattern <= m_maxUsefulPatterns; ++iPattern) {
        /// always show first (all white, used to stablish the noise threshold)
        /// but skip all up to firstPatternToShow
        if (iPattern && iPattern<=firstPatternToShow) {
            continue;
        }

        setCurrentPattern(iPattern);

        for (unsigned int inverted=0; inverted<2; ++inverted) {
            setInverted(inverted != 0);

            QCoreApplication::processEvents();
            QCoreApplication::processEvents();

            QThread::msleep(m_delayMs);

            QCoreApplication::processEvents();
            QCoreApplication::processEvents();

            QString fileName = QString("%1_%2%3.png")
                    .arg(m_useGrayBinary?"gray":"binary")
                    .arg(iPattern, 2, 10, QLatin1Char('0'))
                    .arg(inverted?"_inv":"");

            emit acquireSingle(fileName);
        }
    }

    qDebug() << "acquisition finished in" << acquisitionTime.elapsed() << "msecs";

    emit finishAcquisition();

    /// create the pattern that was just projected
    std::map<int, std::vector<cv::Vec2f> > fringePoints;
    const auto geometry = m_dlpview->geometry();
    const auto projectionHeight = geometry.height();
    const auto projectionWidth = geometry.width();
    const int fringeStep = int(pow(2, firstPatternToShow));
    if (m_vertical) {
        for (int x = fringeStep-1; x<projectionWidth-1; x+=fringeStep) {
            std::vector<cv::Vec2f> fringe;
            fringe.reserve(projectionHeight);
            for (int y = 0; y<projectionHeight; ++y) {
                fringe.push_back(cv::Vec2f(0.5f+x, y));
            }
            fringePoints[x] = fringe;
        }
    } else {
        for (int y = fringeStep-1; y<projectionHeight-1; y+=fringeStep) {
            std::vector<cv::Vec2f> fringe;
            fringe.reserve(projectionWidth);
            for (int x = 0; x<projectionWidth; ++x) {
                fringe.push_back(cv::Vec2f(x, 0.5f+y));
            }
            fringePoints[y] = fringe;
        }
    }
    qDebug() << "pattern has" << fringePoints.size() << "fringes";

    const Z3D::ZProjectedPatternPtr pattern(new Z3D::ZProjectedPattern(cv::Mat(), fringePoints));

    /// notify possible listeners
    emit patternProjected(pattern);

    /// return previous state
    setPreviewEnabled(previewWasEnabled);
}

void ZBinaryPatternProjection::processImages(std::vector<std::vector<ZCameraImagePtr> > acquiredImages, QString scanId)
{
    /// acquiredImages indexing
    ///     1st index: image number / order
    ///     2nd index: camera index
    auto numImages = acquiredImages.size();
    auto numPatterns = numImages / 2;
    auto numCameras = acquiredImages.front().size();

    /// allImages indexing
    ///     1st index: camera
    ///     2nd index: normal/inverted
    ///     3rd index: image number
    std::vector< std::vector< std::vector<cv::Mat> > > allImages;

    allImages.resize(numCameras);
    for (unsigned int iCam=0; iCam<numCameras; ++iCam) {
        allImages[iCam].resize(2);
        allImages[iCam][0].resize(numPatterns); // images
        allImages[iCam][1].resize(numPatterns); // inverted images

        for (unsigned int iPattern=0; iPattern<numPatterns; ++iPattern) {
            int imageIndex = 2 * iPattern;
            allImages[iCam][0][iPattern] = acquiredImages[imageIndex  ][iCam]->cvMat();
            allImages[iCam][1][iPattern] = acquiredImages[imageIndex+1][iCam]->cvMat();
        }
    }

    /// decodification time
    QTime decodeTime;
    decodeTime.start();

    std::vector<Z3D::ZDecodedPatternPtr> decodedPatternList;

    for (unsigned int iCam=0; iCam<numCameras; ++iCam) {
        /// decode images

        auto &cameraImages = allImages[iCam];
        auto &whiteImages = cameraImages[0];
        auto &inverseImages = cameraImages[1];

        /// the first images are the all white, all black
        /// they are used to create the mask of valid pixels
        const cv::Mat &whiteImg = whiteImages.front();
        const cv::Mat &inverseImg = inverseImages.front();

        cv::Mat maskImg = whiteImg - inverseImg;
        /// set mask to keep only values greater than threshold
        maskImg = maskImg > m_noiseThreshold;

        auto intensityImg = whiteImg.clone();

        /// the first images are useless to decode pattern
        whiteImages.erase(whiteImages.begin());
        inverseImages.erase(inverseImages.begin());

        /// decode binary pattern using the rest of the images
        cv::Mat decoded = Z3D::ZBinaryPatternDecoder::decodeBinaryPatternImages(whiteImages, inverseImages, maskImg, m_useGrayBinary);

        if (m_debugMode) {
            cv::imwrite(qPrintable(QString("%1/decoded_%2.tiff")
                                   .arg(scanId)
                                   .arg(iCam)),
                        decoded);
        }

        /// vector of points for every fringe
        std::map<int, std::vector<cv::Vec2f> > fringePoints;
        Z3D::ZBinaryPatternDecoder::simplifyBinaryPatternData(decoded, maskImg, fringePoints);

        Z3D::ZDecodedPatternPtr decodedPattern(new Z3D::ZDecodedPattern(decoded, intensityImg, fringePoints));
        decodedPatternList.push_back(decodedPattern);
    }

    qDebug() << "pattern decodification finished in" << decodeTime.elapsed() << "msecs";

    /// notify result
    emit patternsDecoded(decodedPatternList);
}

double ZBinaryPatternProjection::intensity()
{
    return m_intensity;
}

void ZBinaryPatternProjection::setIntensity(double arg)
{
    if (m_intensity == arg) {
        return;
    }

    m_intensity = arg;
    emit intensityChanged(arg);
}

int ZBinaryPatternProjection::currentPattern()
{
    return m_currentPattern;
}

void ZBinaryPatternProjection::setCurrentPattern(int arg)
{
    if (m_currentPattern == arg) {
        return;
    }

    m_currentPattern = arg;
    emit currentPatternChanged(arg);
}

bool ZBinaryPatternProjection::inverted()
{
    return m_inverted;
}

void ZBinaryPatternProjection::setInverted(bool arg)
{
    if (m_inverted == arg) {
        return;
    }

    m_inverted = arg;
    emit invertedChanged(arg);
}

bool ZBinaryPatternProjection::vertical()
{
    return m_vertical;
}

void ZBinaryPatternProjection::setVertical(bool arg)
{
    if (m_vertical == arg) {
        return;
    }

    m_vertical = arg;
    emit verticalChanged(arg);
    updateMaxUsefulPatterns();
}

bool ZBinaryPatternProjection::useGrayBinary()
{
    return m_useGrayBinary;
}

void ZBinaryPatternProjection::setUseGrayBinary(bool arg)
{
    if (m_useGrayBinary == arg) {
        return;
    }

    m_useGrayBinary = arg;
    emit useGrayBinaryChanged(arg);
}

int ZBinaryPatternProjection::delayMs() const
{
    return m_delayMs;
}

void ZBinaryPatternProjection::setDelayMs(int arg)
{
    if (m_delayMs == arg) {
        return;
    }

    m_delayMs = arg;
    emit delayMsChanged(arg);
}

int ZBinaryPatternProjection::noiseThreshold() const
{
    return m_noiseThreshold;
}

void ZBinaryPatternProjection::setNoiseThreshold(int arg)
{
    if (m_noiseThreshold == arg) {
        return;
    }

    m_noiseThreshold = arg;
    emit noiseThresholdChanged(arg);
}

int ZBinaryPatternProjection::numPatterns() const
{
    return m_numPatterns;
}

void ZBinaryPatternProjection::setNumPatterns(int arg)
{
    if (m_numPatterns == arg) {
        return;
    }

    m_numPatterns = arg;
    emit numPatternsChanged(arg);

    if (!m_automaticPatternCount) {
        int firstPatternToShow = m_maxUsefulPatterns - m_numPatterns;
        qDebug() << "not using automatic pattern count. maxUsefulPatterns:" << m_maxUsefulPatterns;
        qDebug() << "numPatterns:" << m_numPatterns << "firstPatternToShow:" << firstPatternToShow;
    }
}

bool ZBinaryPatternProjection::debugMode() const
{
    return m_debugMode;
}

void ZBinaryPatternProjection::setDebugMode(bool arg)
{
    if (m_debugMode == arg) {
        return;
    }

    m_debugMode = arg;
    emit debugModeChanged(arg);
}

bool ZBinaryPatternProjection::previewEnabled() const
{
    return m_previewEnabled;
}

bool ZBinaryPatternProjection::setPreviewEnabled(bool arg)
{
    bool previousPreviewState = m_previewEnabled;

    if (m_previewEnabled != arg) {
        m_previewEnabled = arg;
        emit previewEnabledChanged(arg);
    }

    if (m_previewEnabled)
        showProjectionWindow();
    else
        hideProjectionWindow();

    return previousPreviewState;
}

bool ZBinaryPatternProjection::automaticPatternCount() const
{
    return m_automaticPatternCount;
}

void ZBinaryPatternProjection::setAutomaticPatternCount(bool arg)
{
    if (m_automaticPatternCount != arg) {
        m_automaticPatternCount = arg;
        emit automaticPatternCountChanged(arg);
    }

    if (m_automaticPatternCount) {
        setNumPatterns(m_maxUsefulPatterns);
    }
}

void ZBinaryPatternProjection::updateMaxUsefulPatterns()
{
    const auto& geometry = m_dlpview->geometry();

    /// to skip useless patterns
    /// log2(number) == log(number)/log(2)
    if (m_vertical) {
        m_maxUsefulPatterns = int(ceil(log(double(geometry.width()))/log(2.)));
    } else {
        m_maxUsefulPatterns = int(ceil(log(double(geometry.height()))/log(2.)));
    }

    qDebug() << "geometry:" << geometry << "maxUsefulPatterns:" << m_maxUsefulPatterns;

    setAutomaticPatternCount(m_automaticPatternCount);
}

} // namespace Z3D
