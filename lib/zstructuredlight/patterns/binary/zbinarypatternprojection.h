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

#include "zpatternprojection.h"

class QQuickView;

namespace Z3D
{

class ZBinaryPatternProjectionConfigWidget;

class ZBinaryPatternProjection : public ZPatternProjection
{
    Q_OBJECT

    Q_PROPERTY(int delayMs READ delayMs WRITE setDelayMs NOTIFY delayMsChanged)
    Q_PROPERTY(int noiseThreshold READ noiseThreshold WRITE setNoiseThreshold NOTIFY noiseThresholdChanged)
    Q_PROPERTY(bool automaticPatternCount READ automaticPatternCount WRITE setAutomaticPatternCount NOTIFY automaticPatternCountChanged)
    Q_PROPERTY(int numPatterns READ numPatterns WRITE setNumPatterns NOTIFY numPatternsChanged)
    Q_PROPERTY(double intensity READ intensity WRITE setIntensity NOTIFY intensityChanged)
    Q_PROPERTY(int currentPattern READ currentPattern WRITE setCurrentPattern NOTIFY currentPatternChanged)
    Q_PROPERTY(bool inverted READ inverted WRITE setInverted NOTIFY invertedChanged)
    Q_PROPERTY(bool vertical READ vertical WRITE setVertical NOTIFY verticalChanged)
    Q_PROPERTY(bool useGrayBinary READ useGrayBinary WRITE setUseGrayBinary NOTIFY useGrayBinaryChanged)
    Q_PROPERTY(bool debugMode READ debugMode WRITE setDebugMode NOTIFY debugModeChanged)
    Q_PROPERTY(bool previewEnabled READ previewEnabled WRITE setPreviewEnabled NOTIFY previewEnabledChanged)

public:
    explicit ZBinaryPatternProjection(QObject *parent = nullptr);
    ~ZBinaryPatternProjection() override;

public:
    // ZPatternProjection interface
    virtual const std::vector<ZSettingsItemPtr> &settings() override;

signals:
    void intensityChanged(double);
    void currentPatternChanged(int);
    void invertedChanged(bool);
    void verticalChanged(bool);
    void useGrayBinaryChanged(bool);
    void delayMsChanged(int arg);
    void noiseThresholdChanged(int arg);
    void numPatternsChanged(int arg);
    void debugModeChanged(bool arg);
    void previewEnabledChanged(bool arg);
    void automaticPatternCountChanged(bool arg);

public slots:
    // ZPatternProjection interface
    virtual void beginScan() override;
    virtual void processImages(std::vector< std::vector<Z3D::ZCameraImagePtr> > acquiredImages, QString scanId) override;

    void showProjectionWindow();
    void hideProjectionWindow();
    void setProjectionWindowGeometry(const QRect &geometry);

    double intensity();
    bool setIntensity(double arg);

    int currentPattern();
    bool setCurrentPattern(int arg);

    bool inverted();
    bool setInverted(bool arg);

    bool vertical();
    bool setVertical(bool arg);

    bool useGrayBinary();
    bool setUseGrayBinary(bool arg);

    int delayMs() const;
    bool setDelayMs(int arg);

    int noiseThreshold() const;
    bool setNoiseThreshold(int arg);

    int numPatterns() const;
    bool setNumPatterns(int arg);

    bool debugMode() const;
    bool setDebugMode(bool arg);

    bool previewEnabled() const;
    bool setPreviewEnabled(bool arg);

    bool automaticPatternCount() const;
    bool setAutomaticPatternCount(bool arg);

private slots:
    bool setSelectedScreen(int index);

protected:
    void updateMaxUsefulPatterns();

    QQuickView *m_dlpview;

    int m_delayMs;
    int m_noiseThreshold;
    bool m_automaticPatternCount;
    int m_numPatterns;
    double m_intensity;
    int m_currentPattern;
    bool m_inverted;
    bool m_vertical;
    bool m_useGrayBinary;

    bool m_debugMode;
    bool m_previewEnabled;

    int m_maxUsefulPatterns;

    std::vector<ZSettingsItemPtr> m_settings;
};

} // namespace Z3D
