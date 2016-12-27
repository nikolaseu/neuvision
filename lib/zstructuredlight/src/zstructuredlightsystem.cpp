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

#include "zstructuredlightsystem.h"

#include "zimageviewer.h"

#include <QTimer>

namespace Z3D
{

ZStructuredLightSystem::ZStructuredLightSystem(QObject *parent)
    : QObject(parent)
    , m_acqManager(nullptr)
    , m_patternProjection(nullptr)
    , m_ready(false)
    , m_debugSaveFringePoints(false)
    , m_debugShowDecodedImages(false)
    , m_debugShowFringes(false)
{

}

bool ZStructuredLightSystem::ready() const
{
    return m_ready;
}

bool ZStructuredLightSystem::debugSaveFringePoints() const
{
    return m_debugSaveFringePoints;
}

bool ZStructuredLightSystem::debugShowDecodedImages() const
{
    return m_debugShowDecodedImages;
}

bool ZStructuredLightSystem::debugShowFringes() const
{
    return m_debugShowFringes;
}

bool ZStructuredLightSystem::start()
{
    if (!m_acqManager || !m_patternProjection)
        return false;

    QTimer::singleShot(0, m_patternProjection, &ZPatternProjection::beginScan);

    return true;
}

void ZStructuredLightSystem::setAcquisitionManager(ZCameraAcquisitionManager *acquisitionManager)
{
    if (m_acqManager == acquisitionManager)
        return;

    discardConnections();

    m_acqManager = acquisitionManager;

    setupConnections();
}

void ZStructuredLightSystem::setPatternProjection(ZPatternProjection *patternProjection)
{
    if (m_patternProjection == patternProjection)
        return;

    discardConnections();

    m_patternProjection = patternProjection;

    setupConnections();
}

void ZStructuredLightSystem::setReady(bool ready)
{
    if (m_ready == ready)
        return;

    m_ready = ready;
    emit readyChanged(ready);
}

void ZStructuredLightSystem::setDebugSaveFringePoints(bool debugSaveFringePoints)
{
    if (m_debugSaveFringePoints == debugSaveFringePoints)
        return;

    m_debugSaveFringePoints = debugSaveFringePoints;
    emit debugSaveFringePointsChanged(debugSaveFringePoints);
}

void ZStructuredLightSystem::setDebugShowDecodedImages(bool debugShowDecodedImages)
{
    if (m_debugShowDecodedImages == debugShowDecodedImages)
        return;

    m_debugShowDecodedImages = debugShowDecodedImages;
    emit debugShowDecodedImagesChanged(debugShowDecodedImages);
}

void ZStructuredLightSystem::setDebugShowFringes(bool debugShowFringes)
{
    if (m_debugShowFringes == debugShowFringes)
        return;

    m_debugShowFringes = debugShowFringes;
    emit debugShowFringesChanged(debugShowFringes);
}

void ZStructuredLightSystem::onPatternsDecodedDebug(std::vector<ZDecodedPattern::Ptr> patterns)
{
    if (m_debugShowDecodedImages || m_debugShowFringes) {
        double minVal,
               maxVal,
               absMinVal = DBL_MAX,
               absMaxVal = DBL_MIN;

        /// only to show in the window, we need to change image "range" to improve visibility
        for (const auto &decodedPattern : patterns) {
            cv::Mat &decoded = decodedPattern->decodedImage;

            ///find minimum and maximum intensities
            minMaxLoc(decoded, &minVal, &maxVal);

            /// minimum will always be zero, we need to find the minimum != 0
            /// we set maxVal where value is zero
            cv::Mat mask = decoded == 0;
            decoded.setTo(cv::Scalar(maxVal), mask);

            /// find correct minimum and maximum intensities (skipping zeros)
            minMaxLoc(decoded, &minVal, &maxVal);

            /// return back to original
            decoded.setTo(cv::Scalar(0), mask);

            if (absMinVal > minVal)
                absMinVal = minVal;
            if (absMaxVal < maxVal)
                absMaxVal = maxVal;
        }

        /// range of values
        double range = (absMaxVal - absMinVal);

        if (m_debugShowDecodedImages) {
            int iCam = 0;
            for (const auto &decodedPattern : patterns) {
                cv::Mat &decodedImage = decodedPattern->decodedImage;

                /// convert to "visible" image to show in window
                cv::Mat decodedVisibleImage;
                decodedImage.convertTo(decodedVisibleImage, CV_8U, 255.0/range, -absMinVal * 255.0/range);

                Z3D::ZImageViewer *imageWidget = new Z3D::ZImageViewer();
                imageWidget->setDeleteOnClose(true);
                imageWidget->setWindowTitle(QString("Decoded image [Camera %1]").arg(iCam++));
                imageWidget->updateImage(decodedVisibleImage);
                imageWidget->show();
            }
        }

        if (m_debugShowFringes) {
            int iCam = 0;
            for (const auto &decodedPattern : patterns) {
                cv::Mat &decodedBorders = decodedPattern->fringeImage;

                /// convert to "visible" image to show in window
                cv::Mat decodedVisibleBorders;
                decodedBorders.convertTo(decodedVisibleBorders, CV_8U, 255.0/range, -absMinVal * 255.0/range);

                Z3D::ZImageViewer *imageWidget = new Z3D::ZImageViewer();
                imageWidget->setDeleteOnClose(true);
                imageWidget->setWindowTitle(QString("Fringe borders [Camera %1]").arg(iCam++));
                imageWidget->updateImage(decodedVisibleBorders);
                imageWidget->show();
            }
        }
    }
}

void ZStructuredLightSystem::setupConnections()
{
    if (!m_acqManager || !m_patternProjection)
        return;

    connect(m_patternProjection, &ZPatternProjection::prepareAcquisition,
            m_acqManager, &ZCameraAcquisitionManager::prepareAcquisition);
    connect(m_patternProjection, &ZPatternProjection::acquireSingle,
            m_acqManager, &ZCameraAcquisitionManager::acquireSingle);
    connect(m_patternProjection, &ZPatternProjection::finishAcquisition,
            m_acqManager, &ZCameraAcquisitionManager::finishAcquisition);

    connect(m_acqManager, &ZCameraAcquisitionManager::acquisitionFinished,
            m_patternProjection, &ZPatternProjection::processImages);

    connect(m_patternProjection, &ZPatternProjection::patternProjected,
            this, &ZStructuredLightSystem::onPatternProjected);
    connect(m_patternProjection, &ZPatternProjection::patternsDecoded,
            this, &ZStructuredLightSystem::onPatternsDecodedDebug);
    connect(m_patternProjection, &ZPatternProjection::patternsDecoded,
            this, &ZStructuredLightSystem::onPatternsDecoded);
}

void ZStructuredLightSystem::discardConnections()
{
    if (!m_acqManager || !m_patternProjection)
        return;

    disconnect(m_patternProjection, &ZPatternProjection::prepareAcquisition,
               m_acqManager, &ZCameraAcquisitionManager::prepareAcquisition);
    disconnect(m_patternProjection, &ZPatternProjection::acquireSingle,
               m_acqManager, &ZCameraAcquisitionManager::acquireSingle);
    disconnect(m_patternProjection, &ZPatternProjection::finishAcquisition,
               m_acqManager, &ZCameraAcquisitionManager::finishAcquisition);

    disconnect(m_acqManager, &ZCameraAcquisitionManager::acquisitionFinished,
               m_patternProjection, &ZPatternProjection::processImages);

    disconnect(m_patternProjection, &ZPatternProjection::patternProjected,
            this, &ZStructuredLightSystem::onPatternProjected);
    disconnect(m_patternProjection, &ZPatternProjection::patternsDecoded,
               this, &ZStructuredLightSystem::onPatternsDecodedDebug);
    disconnect(m_patternProjection, &ZPatternProjection::patternsDecoded,
               this, &ZStructuredLightSystem::onPatternsDecoded);
}

} // namespace Z3D
