#ifndef Z3D_CAMERACALIBRATOR___ZCALIBRATIONPATTERNFINDER_H
#define Z3D_CAMERACALIBRATOR___ZCALIBRATIONPATTERNFINDER_H

#include "zcameracalibrator_global.h"

#include <QObject>
#include <QPointer>
#include <QSharedPointer>

#include <opencv2/core/core.hpp>

namespace Z3D
{

class Z3D_CAMERACALIBRATOR_SHARED_EXPORT ZCalibrationPatternFinder : public QObject
{
    Q_OBJECT

    Q_PROPERTY(int   columns   READ columns   WRITE setColumns   NOTIFY columnsChanged)
    Q_PROPERTY(int   rows      READ rows      WRITE setRows      NOTIFY rowsChanged)
    Q_PROPERTY(float colWidth  READ colWidth  WRITE setColWidth  NOTIFY colWidthChanged)
    Q_PROPERTY(float rowHeight READ rowHeight WRITE setRowHeight NOTIFY rowHeightChanged)

public:
    typedef QSharedPointer<ZCalibrationPatternFinder> Ptr;
    typedef QPointer<ZCalibrationPatternFinder> WeakPtr;

    explicit ZCalibrationPatternFinder(QObject *parent = 0);

    virtual QString name() = 0;

    int columns() const;
    int rows() const;
    double colWidth() const;
    double rowHeight() const;

    QString configHash() const;

    virtual QWidget *configWidget() = 0;

signals:
    void columnsChanged(int columns);
    void rowsChanged(int rows);
    void colWidthChanged(double colWidth);
    void rowHeightChanged(double rowHeight);

    void configHashChanged(QString newConfigHash);

public slots:
    void setColumns(int columns);
    void setRows(int rows);
    void setColWidth(double colWidth);
    void setRowHeight(double rowHeight);

    virtual bool findCalibrationPattern(
            cv::Mat image,
            std::vector<cv::Point2f> &corners,
            std::vector<cv::Point3f> &objectPoints);

protected slots:
    /// returns the hash of the parameters of this class, includign pattern type, id, size, etc
    QString getBaseHash();

    /// subclasses must implement this function to generate a unique "hash" for the current configuration
    virtual void updateConfigHash() = 0;

protected:
    QString m_configHash;

    cv::Size m_boardSize;

    double m_colWidth;
    double m_rowHeight;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZCALIBRATIONPATTERNFINDER_H
