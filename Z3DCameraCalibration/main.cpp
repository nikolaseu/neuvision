#include "mainwindow.h"
#include <QApplication>

#include <QDateTime>
#include <QStyleFactory>
#include <QSplashScreen>
#include <QThread>
#include <QThreadPool>

#include <Z3DCameraProvider>
#include <Z3DCameraCalibrationProvider>
#include "zcalibrationpatternfinderprovider.h"

#include <stdio.h> //stdout

#if QT_VERSION < 0x050000
void LogHandler(QtMsgType type, const char *msg)
{
#else // QT_VERSION >= 0x050000
void LogHandler(QtMsgType type, const QMessageLogContext &context, const QString &msgstr)
{
#endif

    static FILE *m_logFile = 0;
    static QDateTime m_startDateTime = QDateTime::fromTime_t(0); //! 1-1-1970 00:00

    QDateTime currDateTime = QDateTime::currentDateTime();

    if (!m_logFile || m_startDateTime.daysTo( currDateTime ) >= 1) {
        //! if there's no log file or if more than 24hs have passed since previous log started
        m_startDateTime = currDateTime;
        QString m_logFileName;
        if (!m_logFile) {
            //! the first log file has the exact date and time
            m_logFileName = QString("logs/%1___start.txt").arg(m_startDateTime.toString("yyyy-MM-dd_hh-mm-ss"));
            //! set time to 00:00 so after the first one we have one for each day
            m_startDateTime.setTime( QTime(0, 0) );
        } else {
            if (m_logFile != stdout)
                fclose(m_logFile);
            //! set time to 00:00 so after the first one we have one for each day
            m_startDateTime.setTime( QTime(0, 0) );
            m_logFileName = QString("logs/%1.txt").arg(m_startDateTime.toString("yyyy-MM-dd"));
        }
        m_logFile = fopen(qPrintable(m_logFileName), "w");
        if (!m_logFile) {
            m_logFile = stdout;
            fprintf(m_logFile, "ERROR: could not open %s\n", qPrintable(m_logFileName));
        }
    }

    QString debugdate = currDateTime.toString("hh:mm:ss.zzz");
    QString debugType;

    switch (type) {
    case QtDebugMsg:
        debugType = "[D]";
        break;
    case QtWarningMsg:
        debugType = "[W]";
        break;
    case QtCriticalMsg:
        debugType = "[C]";
        break;
    case QtFatalMsg:
        debugType = "[F]";
    }

#if QT_VERSION < 0x050000
    fprintf(m_logFile, "%s %s [0x%.8X]: %s\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), msg);
#else
    fprintf(m_logFile, "%s %s [0x%.8X]: %s\n\t%s:%u\n\t%s\n\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), qPrintable(msgstr), context.file, context.line, context.function);
#endif

    if (m_logFile && m_logFile != stdout) {
        fflush(m_logFile);

#if QT_VERSION < 0x050000
        fprintf(stdout, "%s %s [0x%.8X]: %s\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), msg);
#else
        fprintf(stdout, "%s %s [0x%.8X]: %s\n\t%s:%u\n\t%s\n\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), qPrintable(msgstr), context.file, context.line, context.function);
#endif
    }

    fflush(stdout);
}



int main(int argc, char* argv[])
{
    /// to print out diagnostic information about each plugin it (Qt) tries to load
    //qputenv("QT_DEBUG_PLUGINS", "1");

    ///
    QApplication app(argc,argv);

    /// set message handler for debug
#if QT_VERSION < 0x050000
    qInstallMsgHandler(&LogHandler);
#else
    qInstallMessageHandler(&LogHandler);
#endif

#if QT_VERSION >= 0x050000
    /// set custom style
    QApplication::setStyle(QStyleFactory::create("Fusion"));
/*
    QPalette p;
    p = qApp->palette();
    p.setColor(QPalette::Window, QColor(53,53,53));
    p.setColor(QPalette::Button, QColor(53,53,53));
    p.setColor(QPalette::Highlight, QColor(142,45,197));
    /// text
    p.setColor(QPalette::ButtonText, QColor(255,255,255));
    p.setColor(QPalette::HighlightedText, QColor(255,255,255));
    p.setColor(QPalette::ToolTipText, QColor(255,255,255));
    p.setColor(QPalette::WindowText, QColor(255,255,255));
    qApp->setPalette(p);
*/
#endif

    int result;

    {
        QPixmap pixmap(":/splash.png");
        QSplashScreen splash(pixmap);
        splash.show();

        splash.showMessage("Loading camera acquisition plugins...");
        app.processEvents();
        Z3D::ZCameraProvider::loadPlugins();

        splash.showMessage("Loading camera calibration plugins...");
        app.processEvents();
        Z3D::ZCameraCalibrationProvider::loadPlugins();

        splash.showMessage("Loading calibration pattern finder plugins...");
        app.processEvents();
        Z3D::ZCalibrationPatternFinderProvider::loadPlugins();

        splash.showMessage("Loading main window...");
        app.processEvents();

#ifdef Z3D_THREAD_COUNT_LIMIT
        QThreadPool::globalInstance()->setMaxThreadCount(Z3D_THREAD_COUNT_LIMIT);
#endif

        MainWindow window;
        window.show();

        splash.finish(&window);

        result = app.exec();
    }

    qDebug() << "unloading calibration pattern finder plugins...";
    Z3D::ZCalibrationPatternFinderProvider::unloadPlugins();

    qDebug() << "unloading camera calibration plugins...";
    Z3D::ZCameraCalibrationProvider::unloadPlugins();

    qDebug() << "unloading camera plugins...";
    Z3D::ZCameraProvider::unloadPlugins();

    return result;
}
