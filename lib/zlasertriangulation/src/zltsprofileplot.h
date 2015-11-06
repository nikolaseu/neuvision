#ifndef Z3D_LASERTRIANGULATION___ZLTSPROFILEPLOT_H
#define Z3D_LASERTRIANGULATION___ZLTSPROFILEPLOT_H

#include "zlasertriangulation_global.h"

#include <qwt_plot.h>

class QwtPlotCurve;

namespace Z3D
{

class Z3D_LASERTRIANGULATION_SHARED_EXPORT LTSProfilePlot : public QwtPlot
{
    Q_OBJECT

public:
    explicit LTSProfilePlot(QWidget *parent = 0);

signals:

public slots:
    void setSamples( const QVector<QPointF> &samples );

private:
    QwtPlotCurve *d_curve;
};

} // namespace Z3D

#endif // Z3D_LASERTRIANGULATION___ZLTSPROFILEPLOT_H
