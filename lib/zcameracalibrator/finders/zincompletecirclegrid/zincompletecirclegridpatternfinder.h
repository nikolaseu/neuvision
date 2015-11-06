#ifndef Z3D_CAMERACALIBRATOR___ZINCOMPLETECIRCLEGRIDCALIBRATIONPATTERNFINDER_H
#define Z3D_CAMERACALIBRATOR___ZINCOMPLETECIRCLEGRIDCALIBRATIONPATTERNFINDER_H

#include "zcalibrationpatternfinder.h"

namespace Z3D
{

class ZIncompleteCircleGridPatternFinder : public ZCalibrationPatternFinder
{
    Q_OBJECT

public:
    explicit ZIncompleteCircleGridPatternFinder(QObject *parent = 0);

    virtual QString name();

    virtual QWidget *configWidget();

    int maxColumns() const;
    int maxRows() const;

    bool isAsymmetricGrid() const;

    bool refinePatternPoints() const;

signals:
    void maxColumnsChanged(int columns);
    void maxRowsChanged(int rows);

    void isAsymmetricGridChanged(bool isAsymetric);

    void refinePatternPointsChanged(bool refinePatternPoints);

public slots:
    virtual bool findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &objectPoints);

    void setMaxColumns(int columns);
    void setMaxRows(int rows);

    void setIsAsymmetricGrid(bool isAsymmetric);

    void setRefinePatternPoints(bool refinePatternPoints);

protected slots:
    virtual void updateConfigHash();

protected:
    QWidget *m_configWidget;

    cv::Size m_completeBoardSize;

    bool m_isAsymmetricGrid;

    bool m_refinePatternPoints;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZINCOMPLETECIRCLEGRIDCALIBRATIONPATTERNFINDER_H
