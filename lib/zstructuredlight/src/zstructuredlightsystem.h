#pragma once

#include "zstructuredlight_global.h"
#include "zcameraacquisitionmanager.h"
#include "zdecodedpattern.h"
#include "zpatternprojection.h"
#include "zsimplepointcloud.h"

#include <QObject>

namespace Z3D
{

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZStructuredLightSystem : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool ready READ ready WRITE setReady NOTIFY readyChanged)
    Q_PROPERTY(bool debugSaveFringePoints READ debugSaveFringePoints WRITE setDebugSaveFringePoints NOTIFY debugSaveFringePointsChanged)
    Q_PROPERTY(bool debugShowDecodedImages READ debugShowDecodedImages WRITE setDebugShowDecodedImages NOTIFY debugShowDecodedImagesChanged)
    Q_PROPERTY(bool debugShowFringes READ debugShowFringes WRITE setDebugShowFringes NOTIFY debugShowFringesChanged)

public:
    explicit ZStructuredLightSystem(QObject *parent = 0);

    virtual QString id() const = 0;
    virtual QString displayName() const = 0;

    bool ready() const;
    bool debugSaveFringePoints() const;
    bool debugShowDecodedImages() const;
    bool debugShowFringes() const;

signals:
    void readyChanged(bool ready);
    void debugSaveFringePointsChanged(bool debugSaveFringePoints);
    void debugShowDecodedImagesChanged(bool debugShowDecodedImages);
    void debugShowFringesChanged(bool debugShowFringes);

    void scanFinished(Z3D::ZSimplePointCloud::Ptr cloud);

public slots:
    bool start();

    virtual void setAcquisitionManager(Z3D::ZCameraAcquisitionManager *acquisitionManager);
    virtual void setPatternProjection(Z3D::ZPatternProjection *patternProjection);

    void setReady(bool ready);
    void setDebugSaveFringePoints(bool debugSaveFringePoints);
    void setDebugShowDecodedImages(bool debugShowDecodedImages);
    void setDebugShowFringes(bool debugShowFringes);

    virtual QWidget *configWidget() = 0;

protected slots:
    virtual void onPatternProjected(Z3D::ZProjectedPattern::Ptr pattern) = 0;
    virtual void onPatternsDecoded(std::vector<Z3D::ZDecodedPattern::Ptr> patterns) = 0;

private slots:
    void onPatternsDecodedDebug(std::vector<Z3D::ZDecodedPattern::Ptr> patterns);

private:
    void setupConnections();
    void discardConnections();

    QPointer<Z3D::ZCameraAcquisitionManager> m_acqManager;
    QPointer<Z3D::ZPatternProjection> m_patternProjection;

    bool m_ready;

    bool m_debugSaveFringePoints;
    bool m_debugShowDecodedImages;
    bool m_debugShowFringes;
};

}
