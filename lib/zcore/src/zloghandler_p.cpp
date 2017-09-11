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

#include "zloghandler_p.h"

#include <QDateTime>
#include <QThread>

namespace Z3D
{

void ZLogHandler(QtMsgType type, const QMessageLogContext &context, const QString &msgstr)
{
    Q_UNUSED(context)

    static FILE *m_logFile = nullptr;
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

#if defined(Z3D_RELEASE)
        fprintf(m_logFile, "%s %s [0x%.8X]: %s\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), qPrintable(msgstr));
#else
        fprintf(m_logFile, "%s %s [0x%.8X]: %s\n\t%s:%u\n\t%s\n\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), qPrintable(msgstr), context.file, context.line, context.function);
#endif

    if (m_logFile && m_logFile != stdout) {
        fflush(m_logFile);

#if defined(Z3D_RELEASE)
        fprintf(stdout, "%s %s [0x%.8X]: %s\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), qPrintable(msgstr));
#else
        fprintf(stdout, "%s %s [0x%.8X]: %s\n\t%s:%u\n\t%s\n\n", qPrintable(debugType), qPrintable(debugdate), (long)QThread::currentThread(), qPrintable(msgstr), context.file, context.line, context.function);
#endif
    }

    fflush(stdout);
}

} // namespace Z3D
