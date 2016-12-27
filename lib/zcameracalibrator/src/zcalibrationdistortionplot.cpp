//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "zcalibrationdistortionplot.h"

#include "zcalibrationdistortionplotdata.h"

#include <qwt_color_map.h>
#include <qwt_scale_draw.h>
#include <qwt_scale_engine.h>
#include <qwt_scale_widget.h>
//#include <qwt_plot_grid.h>
#include <qwt_plot_layout.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_spectrogram.h>
#include <qwt_plot_renderer.h>
#include <qwt_plot_rescaler.h>
#include <qwt_plot_zoomer.h>

#include <QTimer>

namespace Z3D
{

class MyZoomer: public QwtPlotZoomer
{
public:
    MyZoomer(ZCameraCalibration *calibration, QWidget *canvas ):
        QwtPlotZoomer( canvas ),
        m_calibration(calibration)
    {
        setTrackerMode( AlwaysOn );
    }

    virtual QwtText trackerTextF( const QPointF &pos ) const
    {
        QColor bg( Qt::white );
        bg.setAlpha( 200 );

        int x = pos.x();
        int y = pos.y();
        if (x<0 || y<0 || x>=m_calibration->sensorWidth() || y>=m_calibration->sensorHeight())
            return QwtText();

        float cx, cy;
        m_calibration->getCalibratedPixel(x, y, &cx, &cy);
        float distortion = sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y));

        QwtText text(QString("(%1,%2) %3 px")
                     .arg(x)
                     .arg(y)
                     .arg(QString::number(distortion, 'f', 3))); // = QwtPlotZoomer::trackerTextF( pos );
        text.setBackgroundBrush( QBrush( bg ) );
        return text;
    }

    ZCameraCalibration *m_calibration;
};




#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))
#define clamp(a, low, high) ((a > low) ? (a < high ? a : high) : low)

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Returns the appropriate color from the "jet" colormap
///
/// Implementing a Continuous "Jet" Colormap Function in GLSL
/// http://www.metastine.com/?p=7
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void setColor(unsigned char* pix, float value)
{
    //    if (value < 0 || value > 1) {
    //        qWarning() << "¿por que no esta entre 0 y 1? " << value;
    //    }

    float valuex4 = value * 4.f;
    float r = min(valuex4 - 1.5f, -valuex4 + 4.5f);
    float g = min(valuex4 - 0.5f, -valuex4 + 3.5f);
    float b = min(valuex4 + 0.5f, -valuex4 + 2.5f);

    pix[0] = (unsigned char) 255.f * clamp(r, 0.f, 1.f); //R
    pix[1] = (unsigned char) 255.f * clamp(g, 0.f, 1.f); //G
    pix[2] = (unsigned char) 255.f * clamp(b, 0.f, 1.f); //B
}


class ColorMapJet: public QwtLinearColorMap
{
public:
    ColorMapJet() :
        QwtLinearColorMap(QColor(0,0,189), QColor(132,0,0))
    {
        unsigned char color[3];
        for (float value=0.f; value<=1.f; value+=0.01f) {
            setColor(color, value);
            addColorStop(value, QColor(color[0],color[1],color[2]));
        }
        /*
        double pos;
        pos = 1.0/13.0*1.0;  addColorStop(pos*pos*pos, QColor(0,0,255));
        pos = 1.0/13.0*2.0;  addColorStop(pos*pos*pos, QColor(0,66,255));
        pos = 1.0/13.0*3.0;  addColorStop(pos*pos*pos, QColor(0,132,255));
        pos = 1.0/13.0*4.0;  addColorStop(pos*pos*pos, QColor(0,189,255));
        pos = 1.0/13.0*5.0;  addColorStop(pos*pos*pos, QColor(0,255,255));
        pos = 1.0/13.0*6.0;  addColorStop(pos*pos*pos, QColor(66,255,189));
        pos = 1.0/13.0*7.0;  addColorStop(pos*pos*pos, QColor(132,255,132));
        pos = 1.0/13.0*8.0;  addColorStop(pos*pos*pos, QColor(189,255,66));
        pos = 1.0/13.0*9.0;  addColorStop(pos*pos*pos, QColor(255,255,0));
        pos = 1.0/13.0*10.0; addColorStop(pos*pos*pos, QColor(255,189,0));
        pos = 1.0/13.0*12.0; addColorStop(pos*pos*pos, QColor(255,66,0));
        pos = 1.0/13.0*13.0; addColorStop(pos*pos*pos, QColor(189,0,0));*/
    }
};

ZCalibrationDistortionPlot::ZCalibrationDistortionPlot(ZCameraCalibration::Ptr calibration, QWidget *parent)
    : QwtPlot(parent)
    , m_spectrogram(0)
    , m_calibration(calibration)
{
    canvas()->setStyleSheet("border: 0px;");

    m_spectrogram = new QwtPlotSpectrogram();
    m_spectrogram->setRenderThreadCount( 0 ); // 0==use system specific thread count

    m_spectrogram->setColorMap( new ColorMapJet() );
    //m_spectrogram->setCachePolicy( QwtPlotRasterItem::PaintCache );
    m_spectrogram->setCachePolicy( QwtPlotRasterItem::NoCache );

    // display contours
    m_spectrogram->setDisplayMode( QwtPlotSpectrogram::ContourMode, true );

    enableAxis( QwtPlot::yRight );

    plotLayout()->setAlignCanvasToScales( true );

    // add rescaler to keep aspect ratio and all image visible
    d_rescaler = new QwtPlotRescaler( canvas() );
    d_rescaler->setReferenceAxis( QwtPlot::xBottom );
    d_rescaler->setAspectRatio( QwtPlot::yLeft, 1.0 );
    d_rescaler->setAspectRatio( QwtPlot::yRight, 0.0 );
    d_rescaler->setAspectRatio( QwtPlot::xTop, 0.0 );
    d_rescaler->setIntervalHint( QwtPlot::xBottom, QwtInterval( 0.0, m_calibration->sensorWidth()-1 ) );
    d_rescaler->setIntervalHint( QwtPlot::yLeft, QwtInterval( 0.0, m_calibration->sensorHeight()-1 ) );
    d_rescaler->setRescalePolicy( QwtPlotRescaler::Fitting );
    for ( int axis = 0; axis < QwtPlot::axisCnt; axis++ )
        d_rescaler->setExpandingDirection( QwtPlotRescaler::ExpandBoth );
    d_rescaler->rescale();

    // Avoid jumping when labels with more/less digits
    // appear/disappear when scrolling vertically
    const QFontMetrics fm( axisWidget( QwtPlot::yLeft )->font() );
    QwtScaleDraw *sd = axisScaleDraw( QwtPlot::yLeft );
    sd->setMinimumExtent( fm.width(  QString::number(m_calibration->sensorHeight()-1) ) );

    // MidButton for the panning
    QwtPlotPanner *panner = new QwtPlotPanner( canvas() );
    panner->setAxisEnabled( QwtPlot::yRight, false );
    panner->setMouseButton( Qt::MidButton );

    // LeftButton for the zooming
    // RightButton: zoom out by 1
    // Ctrl+RighButton: zoom out to full size
    QwtPlotZoomer* zoomer = new MyZoomer( m_calibration.data(), canvas() );
    zoomer->setMousePattern( QwtEventPattern::MouseSelect2,
        Qt::RightButton, Qt::ControlModifier );
    zoomer->setMousePattern( QwtEventPattern::MouseSelect3,
        Qt::RightButton );

    const QColor c( Qt::darkBlue );
    zoomer->setRubberBandPen( c );
    zoomer->setTrackerPen( c );

    /*
    // grid
    QwtPlotGrid *grid = new QwtPlotGrid();
    //grid->enableXMin(true);
    grid->setMajorPen( Qt::white, 0, Qt::DotLine );
    grid->setMinorPen( Qt::gray, 0 , Qt::DotLine );
    grid->attach( this );
    */

    // connect to update
    QObject::connect(m_calibration.data(), SIGNAL(calibrationReady()),
                     this, SLOT(reload()));

    if (m_calibration->ready())
        reload();
}

void ZCalibrationDistortionPlot::showContour( bool on )
{
    m_spectrogram->setDisplayMode( QwtPlotSpectrogram::ContourMode, on );

    replot();
}

void ZCalibrationDistortionPlot::showSpectrogram( bool on )
{
    m_spectrogram->setDisplayMode( QwtPlotSpectrogram::ImageMode, on );
    m_spectrogram->setDefaultContourPen(
        on ? QPen( Qt::black, 0 ) : QPen( Qt::NoPen ) );

    replot();
}

void ZCalibrationDistortionPlot::setAlpha( int alpha )
{
    m_spectrogram->setAlpha( alpha );

    replot();
}

void ZCalibrationDistortionPlot::reload()
{
    qDebug() << Q_FUNC_INFO;

    /// check if calibration is ready, if not, try again later
    if (!m_calibration->ready()) {
        qDebug() << "calibration not ready. will try again later, when the calibration gets ready...";
        //QTimer::singleShot(100, this, SLOT(reload()));
        return;
    }

    /// setData deletes previous data, no need to delete ourselves
    ZCalibrationDistortionData *m_distortionData = new ZCalibrationDistortionData( m_calibration.data() );

    m_spectrogram->setData( m_distortionData );
    m_spectrogram->attach( this );

    // set axis to floating so the plot doesn't have empty space
    m_spectrogram->plot()->axisScaleEngine(QwtPlot::xBottom)->setAttribute(QwtScaleEngine::Floating, true);
    m_spectrogram->plot()->axisScaleEngine(QwtPlot::yLeft)->setAttribute(QwtScaleEngine::Floating, true);

    // contours plot
    QList<double> contourLevels;
#if 0
    const int levels = 24;
    /// total 24 levels between 0 and maxDistortion
    for ( double level = 0; level < levels; level++ )
        contourLevels << std::pow(level/levels, 3.) * m_distortionData->m_maxDistortion;
#else
    if (m_distortionData->m_maxDistortion < 50) {
        /// one contour line for every integer
        for ( double level = 1; level < m_distortionData->m_maxDistortion; level++ )
            contourLevels << level;
    } else {
        const int levels = 24;
        /// total 24 levels between 0 and maxDistortion
        for ( double level = 0; level < levels; level++ )
            contourLevels << level/levels * m_distortionData->m_maxDistortion;
    }
#endif
    m_spectrogram->setContourLevels( contourLevels );

    // add color bar on the right axis
    const QwtInterval zInterval = m_spectrogram->data()->interval( Qt::ZAxis );
    QwtScaleWidget *rightAxis = axisWidget( QwtPlot::yRight );
    rightAxis->setTitle( "Distortion [px]" );
    rightAxis->setColorBarEnabled( true );
    rightAxis->setColorMap( zInterval, new ColorMapJet() );
    setAxisScale( QwtPlot::yRight, zInterval.minValue(), zInterval.maxValue() );
/*
    // set axis limits
    m_spectrogram->data()->setInterval( Qt::XAxis, QwtInterval(0, m_calibration->sensorWidth()-1));
    m_spectrogram->data()->setInterval( Qt::YAxis, QwtInterval(0, m_calibration->sensorHeight()-1));
    setAxisScale( QwtPlot::xBottom, 0, m_calibration->sensorWidth() );
    setAxisScale( QwtPlot::yLeft, 0, m_calibration->sensorHeight() );
*/
    // update rescaler
    d_rescaler->setIntervalHint( QwtPlot::xBottom, QwtInterval( 0.0, m_calibration->sensorWidth()-1 ) );
    d_rescaler->setIntervalHint( QwtPlot::yLeft, QwtInterval( 0.0, m_calibration->sensorHeight()-1 ) );
    d_rescaler->rescale();

    // replot
    replot();
}

} // namespace Z3D
