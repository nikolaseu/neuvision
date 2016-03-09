#pragma once

#include <QDebug>

namespace Z3D {

#if QT_VERSION < 0x050000
void ZLogHandler(QtMsgType type, const char *msg);
#else // QT_VERSION >= 0x050000
void ZLogHandler(QtMsgType type, const QMessageLogContext &context, const QString &msgstr);
#endif

} // namespace Z3D
