#include "zloghandler_p.h"

#include <QDateTime>
#include <QThread>

namespace Z3D
{

#if QT_VERSION < 0x050000
void ZLogHandler(QtMsgType type, const char *msg)
{
#else // QT_VERSION >= 0x050000
void ZLogHandler(QtMsgType type, const QMessageLogContext &context, const QString &msgstr)
{
    Q_UNUSED(context)
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
    case QtInfoMsg:
        debugType = "[I]";
        break;
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
#if defined(Z3D_RELEASE)
        fprintf(m_logFile, "%s %s [0x%.8X]: %s\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), qPrintable(msgstr));
#else
        fprintf(m_logFile, "%s %s [0x%.8X]: %s\n\t%s:%u\n\t%s\n\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), qPrintable(msgstr), context.file, context.line, context.function);
#endif
#endif

    if (m_logFile && m_logFile != stdout) {
        fflush(m_logFile);

#if QT_VERSION < 0x050000
        fprintf(stdout, "%s %s [0x%.8X]: %s\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), msg);
#else
#if defined(Z3D_RELEASE)
        fprintf(stdout, "%s %s [0x%.8X]: %s\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), qPrintable(msgstr));
#else
        fprintf(stdout, "%s %s [0x%.8X]: %s\n\t%s:%u\n\t%s\n\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), qPrintable(msgstr), context.file, context.line, context.function);
#endif
#endif
    }

    fflush(stdout);
}

} // namespace Z3D
