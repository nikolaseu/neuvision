#pragma once

#include <QtCore/QDebug>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#define Z3D_LOGGING_CATEGORY_FROM_FILE(module, minLogType) \
    namespace { \
    const QLoggingCategory &loggingCategory() \
    { \
        static const QLoggingCategory category = []{ \
            const QFileInfo file(__FILE__);\
            const QString categoryName = QString("%1.%2").arg(module).arg(file.baseName()); \
            static const QByteArray categoryName2 = categoryName.toUtf8(); \
            return QLoggingCategory(categoryName2.data(), minLogType); \
        }(); \
        return category; \
    } \
    [[maybe_unused]] auto zDebug = [](){ return qDebug(loggingCategory); }; \
    [[maybe_unused]] auto zInfo = [](){ return qInfo(loggingCategory); }; \
    [[maybe_unused]] auto zWarning = [](){ return qWarning(loggingCategory); }; \
    [[maybe_unused]] auto zCritical = [](){ return qCritical(loggingCategory); }; \
    }
