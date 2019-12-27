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

#include "ZStructuredLight/zstructuredlight_fwd.h"
#include "ZStructuredLight/zstructuredlight_global.h"

#include "ZCore/zcore_fwd.h"
#include "ZPointCloud/zpointcloud_fwd.h"

#include <QObject>

class QSettings;

namespace Z3D
{

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZStructuredLightSystem : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool ready READ ready WRITE setReady NOTIFY readyChanged)
    Q_PROPERTY(bool debugShowDecodedImages READ debugShowDecodedImages WRITE setDebugShowDecodedImages NOTIFY debugShowDecodedImagesChanged)

public:
    explicit ZStructuredLightSystem(const ZCameraAcquisitionManagerPtr &acquisitionManager,
                                    const ZPatternProjectionPtr &patternProjection,
                                    QObject *parent = nullptr);

    virtual ~ZStructuredLightSystem();

    virtual const std::vector<ZSettingsItemPtr> &settings() = 0;

    bool ready() const;
    bool debugShowDecodedImages() const;

    ZPatternProjectionPtr patternProjection() const;

signals:
    void readyChanged(bool ready);
    void debugShowDecodedImagesChanged(bool debugShowDecodedImages);

    void scanFinished(Z3D::ZPointCloudPtr cloud);

public slots:
    bool start();

    void setReady(bool ready);
    bool setDebugShowDecodedImages(bool debugShowDecodedImages);

protected slots:
    virtual void onPatternProjected(const Z3D::ZProjectedPatternPtr &pattern) = 0;
    virtual void onPatternsDecoded(const std::vector<Z3D::ZDecodedPatternPtr> &patterns) = 0;

private slots:
    void onPatternsDecodedDebug(const std::vector<Z3D::ZDecodedPatternPtr> &patterns);

private:
    void setupConnections();
    void discardConnections();

    ZCameraAcquisitionManagerPtr m_acqManager;
    ZPatternProjectionPtr m_patternProjection;

    bool m_ready;

    bool m_debugShowDecodedImages;
};

}
