#ifndef ZSTRUCTUREDLIGHTSYSTEM_H
#define ZSTRUCTUREDLIGHTSYSTEM_H

#include "zcameraacquisitionmanager.h"
#include "zdecodedpattern.h"
#include "zpatternprojection.h"
#include "zsimplepointcloud.h"

#include <QObject>

namespace Z3D
{

class ZStructuredLightSystem : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool ready READ ready WRITE setReady NOTIFY readyChanged)

public:
    explicit ZStructuredLightSystem(QObject *parent = 0);

    virtual QString displayName() = 0;

    bool ready() const;

signals:
    void scanFinished(Z3D::ZSimplePointCloud::Ptr cloud);
    void readyChanged(bool ready);

public slots:
    bool start();

    virtual void setAcquisitionManager(Z3D::ZCameraAcquisitionManager *acquisitionManager);
    virtual void setPatternProjection(Z3D::ZPatternProjection *patternProjection);

    void setReady(bool ready);

    virtual QWidget *configWidget() = 0;

protected slots:
    virtual void onPatternsDecoded(std::vector<Z3D::ZDecodedPattern::Ptr> pattern) = 0;

private:
    void setupConnections();
    void discardConnections();

    QPointer<Z3D::ZCameraAcquisitionManager> m_acqManager;
    QPointer<Z3D::ZPatternProjection> m_patternProjection;

    bool m_ready;
};

}

#endif // ZSTRUCTUREDLIGHTSYSTEM_H
