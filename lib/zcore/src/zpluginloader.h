#ifndef Z3D_CORE___ZPLUGINLOADER_H
#define Z3D_CORE___ZPLUGINLOADER_H

#include "zcore_global.h"

#include <QObject>

namespace Z3D
{

class Z3D_CORE_SHARED_EXPORT ZPluginLoader : public QObject
{
    Q_OBJECT

public:
    static ZPluginLoader *instance();

    static void loadPlugins(QString folder = QString());
    static void unloadPlugins();

    static const QList<QObject *> plugins(const QString &pluginType);

signals:

public slots:

private:
    explicit ZPluginLoader(QObject *parent = 0);

    static ZPluginLoader *m_instance;

    std::map<QString, QList<QObject *>> m_plugins;
};

} // namespace Z3D

#endif // Z3D_CORE___ZPLUGINLOADER_H
