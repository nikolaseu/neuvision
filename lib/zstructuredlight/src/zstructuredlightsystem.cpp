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

#include "zcameraacquisitionmanager.h"
#include "zdecodedpattern.h"
#include "zimageviewer.h"
#include "zpatternprojection.h"

#include <opencv2/core.hpp>

#include <QDebug>
#include <QTimer>

namespace Z3D
{

ZStructuredLightSystem::ZStructuredLightSystem(
        ZCameraAcquisitionManagerPtr acquisitionManager,
        ZPatternProjectionPtr patternProjection,
        QObject *parent)
    : QObject(parent)
    , m_acqManager(acquisitionManager)
    , m_patternProjection(patternProjection)
    , m_ready(false)
    , m_debugShowDecodedImages(false)
{
    connect(m_patternProjection.get(), &ZPatternProjection::prepareAcquisition,
            m_acqManager.get(), &ZCameraAcquisitionManager::prepareAcquisition);
    connect(m_patternProjection.get(), &ZPatternProjection::acquireSingle,
            m_acqManager.get(), &ZCameraAcquisitionManager::acquireSingle);
    connect(m_patternProjection.get(), &ZPatternProjection::finishAcquisition,
            m_acqManager.get(), &ZCameraAcquisitionManager::finishAcquisition);

    connect(m_acqManager.get(), &ZCameraAcquisitionManager::acquisitionFinished,
            m_patternProjection.get(), &ZPatternProjection::processImages);

    connect(m_patternProjection.get(), &ZPatternProjection::patternProjected,
            this, &ZStructuredLightSystem::onPatternProjected);
    connect(m_patternProjection.get(), &ZPatternProjection::patternsDecoded,
            this, &ZStructuredLightSystem::onPatternsDecodedDebug);
    connect(m_patternProjection.get(), &ZPatternProjection::patternsDecoded,
            this, &ZStructuredLightSystem::onPatternsDecoded);
}

ZStructuredLightSystem::~ZStructuredLightSystem()
{

}

bool ZStructuredLightSystem::ready() const
{
    return m_ready;
}

bool ZStructuredLightSystem::debugShowDecodedImages() const
{
    return m_debugShowDecodedImages;
}

ZPatternProjectionPtr ZStructuredLightSystem::patternProjection() const
{
    return m_patternProjection;
}

bool ZStructuredLightSystem::start()
{
    QTimer::singleShot(0, m_patternProjection.get(), &ZPatternProjection::beginScan);

    return true;
}

void ZStructuredLightSystem::setReady(bool ready)
{
    if (m_ready == ready) {
        return;
    }

    m_ready = ready;
    emit readyChanged(ready);
}

bool ZStructuredLightSystem::setDebugShowDecodedImages(bool debugShowDecodedImages)
{
    if (m_debugShowDecodedImages == debugShowDecodedImages) {
        return true;
    }

    m_debugShowDecodedImages = debugShowDecodedImages;
    emit debugShowDecodedImagesChanged(debugShowDecodedImages);

    return true;
}

void ZStructuredLightSystem::onPatternsDecodedDebug(std::vector<ZDecodedPatternPtr> patterns)
{
    if (!m_debugShowDecodedImages) {
        return;
    }

    double minVal,
           maxVal,
           absMinVal = DBL_MAX,
           absMaxVal = DBL_MIN;

    /// only to show in the window, we need to change image "range" to improve visibility
    for (const auto &decodedPattern : patterns) {
        cv::Mat decoded = decodedPattern->decodedImage().clone();

        /// find minimum and maximum intensities
        cv::minMaxLoc(decoded, &minVal, &maxVal);

        /// to discard invalid values
        cv::Mat mask = decoded == Z3D::ZDecodedPattern::NO_VALUE;
        decoded.setTo(cv::Scalar(maxVal == Z3D::ZDecodedPattern::NO_VALUE ? minVal : maxVal), mask);

        /// find correct minimum and maximum intensities (skipping zeros)
        cv::minMaxLoc(decoded, &minVal, &maxVal);

        qDebug() << "min:" << minVal
                 << "max:" << maxVal;

        if (absMinVal > minVal) {
            absMinVal = minVal;
        }

        if (absMaxVal < maxVal) {
            absMaxVal = maxVal;
        }
    }

    /// range of values
    double range = (absMaxVal - absMinVal);

    qDebug() << "abs min:" << absMinVal
             << "abs max:" << absMaxVal
             << "range:" << range;

    int iCam = 0;
    for (const auto &decodedPattern : patterns) {
        cv::Mat decodedImage = decodedPattern->decodedImage();

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

} // namespace Z3D
