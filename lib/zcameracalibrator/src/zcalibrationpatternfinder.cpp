#include "zcalibrationpatternfinder.h"

#include <QDebug>

namespace Z3D
{

ZCalibrationPatternFinder::ZCalibrationPatternFinder(QObject *parent)
    : QObject(parent)
    , m_boardSize(15, 11)
    , m_colWidth(2.f)
    , m_rowHeight(2.f)
{
    QObject::connect(this, SIGNAL(columnsChanged(int)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(rowsChanged(int)),
                     this, SLOT(updateConfigHash()));
    /// col width and row height changes do not change the way the pattern finder works, but they change the calibration pattern "world" coordinates
    QObject::connect(this, SIGNAL(colWidthChanged(double)),
                     this, SLOT(updateConfigHash()));
    QObject::connect(this, SIGNAL(rowHeightChanged(double)),
                     this, SLOT(updateConfigHash()));
}

int ZCalibrationPatternFinder::columns() const
{
    return m_boardSize.width;
}

int ZCalibrationPatternFinder::rows() const
{
    return m_boardSize.height;
}

double ZCalibrationPatternFinder::colWidth() const
{
    return m_colWidth;
}

double ZCalibrationPatternFinder::rowHeight() const
{
    return m_rowHeight;
}

QString ZCalibrationPatternFinder::configHash() const
{
    return m_configHash;
}

void ZCalibrationPatternFinder::setColumns(int columns)
{
    if (m_boardSize.width != columns) {
        m_boardSize.width = columns;
        emit columnsChanged(columns);
    }
}

void ZCalibrationPatternFinder::setRows(int rows)
{
    if (m_boardSize.height != rows) {
        m_boardSize.height = rows;
        emit rowsChanged(rows);
    }
}

void ZCalibrationPatternFinder::setColWidth(double colWidth)
{
    if (m_colWidth != colWidth) {
        m_colWidth = colWidth;
        emit colWidthChanged(colWidth);
    }
}

void ZCalibrationPatternFinder::setRowHeight(double rowHeight)
{
    if (m_rowHeight != rowHeight) {
        m_rowHeight = rowHeight;
        emit rowHeightChanged(rowHeight);
    }
}

bool ZCalibrationPatternFinder::findCalibrationPattern(cv::Mat /*image*/, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &objectPoints)
{
    corners.clear();
    objectPoints.clear();

    return false;
}

QString ZCalibrationPatternFinder::getBaseHash()
{
    /// get base "hash" for the current base configuration
    return QString("%1|%2|%3|%4|%5")
            .arg(name())
            .arg(m_boardSize.width)
            .arg(m_boardSize.height)
            .arg(m_colWidth)
            .arg(m_rowHeight);
    /// col width and row height changes do not change the way the pattern finder works, but they change the calibration pattern "world" coordinates
}

} // namespace Z3D
