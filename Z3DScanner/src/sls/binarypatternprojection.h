#ifndef BINARYPATTERNPROJECTION_H
#define BINARYPATTERNPROJECTION_H

#include "decodedpattern.h"

#include <Z3DCalibratedCamera>

#include <QObject>

#if QT_VERSION < 0x050000
class QDeclarativeView;
#else
class QQuickView;
#endif

class BinaryPatternProjectionConfigWidget;


class BinaryPatternProjection : public QObject
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
    explicit BinaryPatternProjection(QObject *parent = 0);
    ~BinaryPatternProjection();

signals:
    void patternDecoded(Z3D::DecodedPattern::Ptr pattern);

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
    void showProjectionWindow();
    void hideProjectionWindow();
    void setProjectionWindowGeometry(const QRect &geometry);

    QWidget *configWidget();

    void scan();

    void setCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameraList);

    double intensity();
    void setIntensity(double arg);

    int currentPattern();
    void setCurrentPattern(int arg);

    bool inverted();
    void setInverted(bool arg);

    bool vertical();
    void setVertical(bool arg);

    bool useGrayBinary();
    void setUseGrayBinary(bool arg);

    int delayMs() const;
    void setDelayMs(int arg);

    int noiseThreshold() const;
    void setNoiseThreshold(int arg);

    int numPatterns() const;
    void setNumPatterns(int arg);

    bool debugMode() const;
    void setDebugMode(bool arg);

    bool previewEnabled() const;
    bool setPreviewEnabled(bool arg);

    bool automaticPatternCount() const;
    void setAutomaticPatternCount(bool arg);

protected:
#if QT_VERSION < 0x050000
    QDeclarativeView *m_dlpview;
#else
    QQuickView *m_dlpview;
#endif

    BinaryPatternProjectionConfigWidget *m_configWidget;

    QList<Z3D::ZCalibratedCamera::Ptr> m_camList;

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
};

#endif // BINARYPATTERNPROJECTION_H
