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

#include "ZCameraAcquisition/zcameraimage.h"
#include "ZCore/zsettingsitem.h"
#include "ZStructuredLight/zdecodedpattern.h"
#include "ZStructuredLight/zprojectedpattern.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QElapsedTimer>
#include <QGuiApplication>
#include <QQmlContext>
#include <QQuickView>
#include <QScreen>
#include <QThread>
#include <opencv2/imgcodecs.hpp>

namespace Z3D
{

ZBinaryPatternProjection::ZBinaryPatternProjection(QObject *parent)
    : ZPatternProjection(parent)
    , m_dlpview(nullptr)
    , m_delayMs(500)
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
    m_dlpview->setFlags(Qt::FramelessWindowHint
                        //| Qt::MaximizeUsingFullscreenGeometryHint
                        | Qt::WindowTransparentForInput
                        | Qt::WindowDoesNotAcceptFocus
                        //| Qt::WindowStaysOnBottomHint
                        //| Qt::WindowStaysOnTopHint
                        );
    m_dlpview->rootContext()->setContextProperty("config", this);
//    m_dlpview->rootContext()->setContextProperty("window", m_dlpview);
    m_dlpview->setSource(QUrl("qrc:///zstructuredlight/qml/binarypatternprojection.qml"));
    m_dlpview->setResizeMode(QQuickView::SizeRootObjectToView);

    const QString projectorScreen("Projector screen");

    std::shared_ptr<ZSettingsItemEnum> screenOption = std::make_unique<ZSettingsItemEnum>(
        projectorScreen,
        "Projector",
        "Screen/projector to use",
        [=]() {
            std::vector<QString> screens;
            for (const auto *screen : qGuiApp->screens()) {
                const auto size = screen->size();
                screens.push_back(
                    QString("%2x%3 [%1]").arg(screen->name()).arg(size.width()).arg(size.height()));
            }
            return screens;
        },
        [=]() { return selectedScreen(); },
        std::bind(&ZBinaryPatternProjection::setSelectedScreen, this, std::placeholders::_1));
    QObject::connect(qGuiApp, &QGuiApplication::screenAdded,
                     screenOption.get(), &ZSettingsItemEnum::optionsChanged);
    QObject::connect(qGuiApp, &QGuiApplication::screenRemoved,
                     screenOption.get(), &ZSettingsItemEnum::optionsChanged);

    const QString preview("Preview");

    ZSettingsItemPtr previewOption = std::make_unique<ZSettingsItemBool>(preview, "Show preview", "Show preview",
                                                                         std::bind(&ZBinaryPatternProjection::previewEnabled, this),
                                                                         std::bind(&ZBinaryPatternProjection::setPreviewEnabled, this, std::placeholders::_1));
    QObject::connect(this, &ZBinaryPatternProjection::previewEnabledChanged,
                     previewOption.get(), &ZSettingsItem::valueChanged);

    ZSettingsItemPtr currentPatternOption = std::make_unique<ZSettingsItemInt>(preview, "Current pattern", "Current pattern being displayed",
                                                                               std::bind(&ZBinaryPatternProjection::currentPattern, this),
                                                                               std::bind(&ZBinaryPatternProjection::setCurrentPattern, this, std::placeholders::_1),
                                                                               0, // minimum
                                                                               11); // maximum 2048 == 11 bits
    QObject::connect(this, &ZBinaryPatternProjection::currentPatternChanged,
                     currentPatternOption.get(), &ZSettingsItem::valueChanged);
    QObject::connect(this, &ZBinaryPatternProjection::previewEnabledChanged,
                     currentPatternOption.get(), &ZSettingsItem::setWritable);
    currentPatternOption->setWritable(previewEnabled());

    ZSettingsItemPtr invertedOption = std::make_unique<ZSettingsItemBool>(preview, "Inverted", "Display inverted pattern (for the currently displayed pattern)",
                                                                          std::bind(&ZBinaryPatternProjection::inverted, this),
                                                                          std::bind(&ZBinaryPatternProjection::setInverted, this, std::placeholders::_1));
    QObject::connect(this, &ZBinaryPatternProjection::invertedChanged,
                     invertedOption.get(), &ZSettingsItem::valueChanged);
    QObject::connect(this, &ZBinaryPatternProjection::previewEnabledChanged,
                     invertedOption.get(), &ZSettingsItem::setWritable);
    invertedOption->setWritable(previewEnabled());

    const QString scanConfiguration("Scan configuration");

    ZSettingsItemPtr useGrayCodeOption = std::make_unique<ZSettingsItemBool>(scanConfiguration, "Use gray binary code", "Use gray code instead of regular binary code",
                                                                             std::bind(&ZBinaryPatternProjection::useGrayBinary, this),
                                                                             std::bind(&ZBinaryPatternProjection::setUseGrayBinary, this, std::placeholders::_1));
    QObject::connect(this, &ZBinaryPatternProjection::useGrayBinaryChanged,
                     useGrayCodeOption.get(), &ZSettingsItem::valueChanged);

    ZSettingsItemPtr useVerticalPatternOption = std::make_unique<ZSettingsItemBool>(scanConfiguration, "Vertical patterns", "Use vertical patterns (for example if the projector is rotated or cameras are arranged vertically)",
                                                                                    std::bind(&ZBinaryPatternProjection::vertical, this),
                                                                                    std::bind(&ZBinaryPatternProjection::setVertical, this, std::placeholders::_1));
    QObject::connect(this, &ZBinaryPatternProjection::verticalChanged,
                     useVerticalPatternOption.get(), &ZSettingsItem::valueChanged);

    ZSettingsItemPtr automaticPatternCountOption = std::make_unique<ZSettingsItemBool>(scanConfiguration, "Automatic pattern count", "Automatically determine number of patterns to display based on projector resolution",
                                                                                       std::bind(&ZBinaryPatternProjection::automaticPatternCount, this),
                                                                                       std::bind(&ZBinaryPatternProjection::setAutomaticPatternCount, this, std::placeholders::_1));
    QObject::connect(this, &ZBinaryPatternProjection::automaticPatternCountChanged,
                     automaticPatternCountOption.get(), &ZSettingsItem::valueChanged);

    ZSettingsItemPtr patternCountOption = std::make_unique<ZSettingsItemInt>(scanConfiguration, "Number of patterns", "Number of binary patterns to use",
                                                                             std::bind(&ZBinaryPatternProjection::numPatterns, this),
                                                                             std::bind(&ZBinaryPatternProjection::setNumPatterns, this, std::placeholders::_1),
                                                                             1, // minimum
                                                                             11); // maximum
    QObject::connect(this, &ZBinaryPatternProjection::numPatternsChanged,
                     patternCountOption.get(), &ZSettingsItem::valueChanged);
    QObject::connect(this, &ZBinaryPatternProjection::automaticPatternCountChanged,
                     patternCountOption.get(), &ZSettingsItem::setNonWritable);
    patternCountOption->setNonWritable(automaticPatternCount());

    const QString advancedSettings("Advanced settings");

    ZSettingsItemPtr delayOption = std::make_unique<ZSettingsItemInt>(advancedSettings, "Delay", "Delay after projecting pattern (in ms)",
                                                                      std::bind(&ZBinaryPatternProjection::delayMs, this),
                                                                      std::bind(&ZBinaryPatternProjection::setDelayMs, this, std::placeholders::_1),
                                                                      0, // minimum
                                                                      10000); // maximum
    QObject::connect(this, &ZBinaryPatternProjection::delayMsChanged,
                     delayOption.get(), &ZSettingsItem::valueChanged);

    ZSettingsItemPtr noiseThresholdOption = std::make_unique<ZSettingsItemInt>(advancedSettings, "Noise threshold", "Discard pixels where intensity doesn't change more than threshold value",
                                                                               std::bind(&ZBinaryPatternProjection::noiseThreshold, this),
                                                                               std::bind(&ZBinaryPatternProjection::setNoiseThreshold, this, std::placeholders::_1),
                                                                               0.0, // minimum
                                                                               255.0); // maximum
    QObject::connect(this, &ZBinaryPatternProjection::noiseThresholdChanged,
                     noiseThresholdOption.get(), &ZSettingsItem::valueChanged);

    ZSettingsItemPtr intensityOption = std::make_unique<ZSettingsItemFloat>(advancedSettings, "Intensity", "Intensity of projected pattern",
                                                                            std::bind(&ZBinaryPatternProjection::intensity, this),
                                                                            std::bind(&ZBinaryPatternProjection::setIntensity, this, std::placeholders::_1),
                                                                            0.0, // minimum
                                                                            1.0); // maximum
    QObject::connect(this, &ZBinaryPatternProjection::intensityChanged,
                     intensityOption.get(), &ZSettingsItem::valueChanged);

    ZSettingsItemPtr saveDebugInfoOption = std::make_unique<ZSettingsItemBool>(advancedSettings, "Save debug info", "Save extra information for debugging purposes",
                                                                               std::bind(&ZBinaryPatternProjection::debugMode, this),
                                                                               std::bind(&ZBinaryPatternProjection::setDebugMode, this, std::placeholders::_1));
    QObject::connect(this, &ZBinaryPatternProjection::debugModeChanged,
                     saveDebugInfoOption.get(), &ZSettingsItem::valueChanged);

    m_settings = {
        screenOption,
        previewOption,
        currentPatternOption,
        invertedOption,
        useGrayCodeOption,
        useVerticalPatternOption,
        automaticPatternCountOption,
        patternCountOption,
        delayOption,
        noiseThresholdOption,
        intensityOption,
        saveDebugInfoOption
    };
}

ZBinaryPatternProjection::~ZBinaryPatternProjection()
{
    qDebug() << "deleting projector window...";
    if (m_dlpview) {
        m_dlpview->deleteLater();
    }
}

const std::vector<ZSettingsItemPtr> &ZBinaryPatternProjection::settings()
{
    return m_settings;
}

void ZBinaryPatternProjection::showProjectionWindow()
{
    m_dlpview->show();
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
    QElapsedTimer acquisitionTime;
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

            QThread::msleep(ulong(m_delayMs/2));

            QCoreApplication::processEvents();
            QCoreApplication::processEvents();

            QString fileName = QString("%1_%2%3.png")
                    .arg(m_useGrayBinary?"gray":"binary")
                    .arg(iPattern, 2, 10, QLatin1Char('0'))
                    .arg(inverted?"_inv":"");

            emit acquireSingle(fileName);

            QCoreApplication::processEvents();
            QCoreApplication::processEvents();

            QThread::msleep(ulong(m_delayMs/2));
        }
    }

    qDebug() << "acquisition finished in" << acquisitionTime.elapsed() << "msecs";

    emit finishAcquisition();

    /// create the pattern that was just projected
    const auto geometry = m_dlpview->geometry();
    const auto projectionHeight = geometry.height();
    const auto projectionWidth = geometry.width();
    cv::Mat projectedPatternImg(projectionHeight, projectionWidth, CV_32FC1);
    const int fringeStep = int(pow(2, firstPatternToShow));
    if (m_vertical) {
        for (int x = 0; x<projectionWidth; x++) {
            const float val = std::floor(float(x)/fringeStep);
            for (int y = 0; y<projectionHeight; ++y) {
                projectedPatternImg.at<float_t>(y, x) = val;
            }
        }
    }
    else {
        for (int y = 0; y<projectionHeight; y++) {
            const float val = std::floor(float(y)/fringeStep);
            for (int x = 0; x<projectionWidth; ++x) {
                projectedPatternImg.at<float_t>(y, x) = val;
            }
        }
    }

    const Z3D::ZProjectedPatternPtr pattern(new Z3D::ZProjectedPattern(projectedPatternImg));

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
    for (size_t iCam=0; iCam<numCameras; ++iCam) {
        allImages[iCam].resize(2);
        allImages[iCam][0].resize(numPatterns); // images
        allImages[iCam][1].resize(numPatterns); // inverted images

        for (size_t iPattern=0; iPattern<numPatterns; ++iPattern) {
            size_t imageIndex = 2 * iPattern;
            allImages[iCam][0][iPattern] = acquiredImages[imageIndex  ][iCam]->cvMat();
            allImages[iCam][1][iPattern] = acquiredImages[imageIndex+1][iCam]->cvMat();
        }
    }

    /// decodification time
    QElapsedTimer decodeTime;
    decodeTime.start();

    std::vector<Z3D::ZDecodedPatternPtr> decodedPatternList;

    for (size_t iCam=0; iCam<numCameras; ++iCam) {
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

        Z3D::ZDecodedPatternPtr decodedPattern(new Z3D::ZDecodedPattern(decoded, intensityImg));
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

bool ZBinaryPatternProjection::setIntensity(double arg)
{
    if (qFuzzyCompare(m_intensity, arg)) {
        return true;
    }

    m_intensity = arg;
    emit intensityChanged(arg);

    return true;
}

int ZBinaryPatternProjection::currentPattern()
{
    return m_currentPattern;
}

bool ZBinaryPatternProjection::setCurrentPattern(int arg)
{
    if (m_currentPattern == arg) {
        return true;
    }

    m_currentPattern = arg;
    emit currentPatternChanged(arg);

    return true;
}

bool ZBinaryPatternProjection::inverted()
{
    return m_inverted;
}

bool ZBinaryPatternProjection::setInverted(bool arg)
{
    if (m_inverted == arg) {
        return true;
    }

    m_inverted = arg;
    emit invertedChanged(arg);

    return true;
}

bool ZBinaryPatternProjection::vertical()
{
    return m_vertical;
}

bool ZBinaryPatternProjection::setVertical(bool arg)
{
    if (m_vertical == arg) {
        return true;
    }

    m_vertical = arg;
    emit verticalChanged(arg);
    updateMaxUsefulPatterns();

    return true;
}

bool ZBinaryPatternProjection::useGrayBinary()
{
    return m_useGrayBinary;
}

bool ZBinaryPatternProjection::setUseGrayBinary(bool arg)
{
    if (m_useGrayBinary == arg) {
        return true;
    }

    m_useGrayBinary = arg;
    emit useGrayBinaryChanged(arg);

    return true;
}

int ZBinaryPatternProjection::delayMs() const
{
    return m_delayMs;
}

bool ZBinaryPatternProjection::setDelayMs(int arg)
{
    if (m_delayMs == arg) {
        return true;
    }

    m_delayMs = arg;
    emit delayMsChanged(arg);

    return true;
}

int ZBinaryPatternProjection::noiseThreshold() const
{
    return m_noiseThreshold;
}

bool ZBinaryPatternProjection::setNoiseThreshold(int arg)
{
    if (m_noiseThreshold == arg) {
        return true;
    }

    m_noiseThreshold = arg;
    emit noiseThresholdChanged(arg);

    return true;
}

int ZBinaryPatternProjection::numPatterns() const
{
    return m_numPatterns;
}

bool ZBinaryPatternProjection::setNumPatterns(int arg)
{
    if (m_numPatterns == arg) {
        return true;
    }

    m_numPatterns = arg;
    emit numPatternsChanged(arg);

    if (!m_automaticPatternCount) {
        int firstPatternToShow = m_maxUsefulPatterns - m_numPatterns;
        qDebug() << "not using automatic pattern count. maxUsefulPatterns:" << m_maxUsefulPatterns;
        qDebug() << "numPatterns:" << m_numPatterns << "firstPatternToShow:" << firstPatternToShow;
    }

    return true;
}

bool ZBinaryPatternProjection::debugMode() const
{
    return m_debugMode;
}

bool ZBinaryPatternProjection::setDebugMode(bool arg)
{
    if (m_debugMode == arg) {
        return true;
    }

    m_debugMode = arg;
    emit debugModeChanged(arg);

    return true;
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

bool ZBinaryPatternProjection::setAutomaticPatternCount(bool arg)
{
    if (m_automaticPatternCount != arg) {
        m_automaticPatternCount = arg;
        emit automaticPatternCountChanged(arg);
    }

    if (m_automaticPatternCount) {
        setNumPatterns(m_maxUsefulPatterns);
    }

    return true;
}

int ZBinaryPatternProjection::selectedScreen() const
{
    const auto screens = qGuiApp->screens();
    for (int i=0; i<screens.size(); ++i) {
        const auto &screen = screens[i];
        if (screen->geometry().contains(m_dlpview->geometry().center())) {
            return i;
        }
    }

    qWarning() << "couldn't find screen being used for pattern projection!";

    return -1;
}

bool ZBinaryPatternProjection::setSelectedScreen(int index)
{
    qDebug() << "setting screen to" << index;

    const QList<QScreen*> screens = qGuiApp->screens();
    const QRect screenGeometry = screens[index]->geometry();
    setProjectionWindowGeometry(screenGeometry);

    return true;
}

void ZBinaryPatternProjection::updateMaxUsefulPatterns()
{
    const auto geometry = m_dlpview->geometry();

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
