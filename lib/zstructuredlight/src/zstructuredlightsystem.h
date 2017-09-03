/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "zstructuredlight_global.h"
#include "zcameraacquisitionmanager.h"
#include "zdecodedpattern.h"
#include "zpatternprojection.h"
#include "zsimplepointcloud.h"

#include <QObject>

QT_BEGIN_NAMESPACE
class QSettings;
QT_END_NAMESPACE

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
    typedef QSharedPointer<ZStructuredLightSystem> Ptr;

    explicit ZStructuredLightSystem(QObject *parent = 0);
    virtual ~ZStructuredLightSystem();

    virtual QString id() const = 0;
    virtual QString displayName() const = 0;

    virtual void init(QSettings *settings) = 0;

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
