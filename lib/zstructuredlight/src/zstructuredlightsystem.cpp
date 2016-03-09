#include "zstructuredlightsystem.h"

#include <QTimer>

namespace Z3D
{

ZStructuredLightSystem::ZStructuredLightSystem(QObject *parent)
    : QObject(parent)
    , m_acqManager(nullptr)
    , m_patternProjection(nullptr)
    , m_ready(false)
{

}

bool ZStructuredLightSystem::ready() const
{
    return m_ready;
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

    disconnect(m_patternProjection, &ZPatternProjection::patternsDecoded,
               this, &ZStructuredLightSystem::onPatternsDecoded);
}

} // namespace Z3D
