#ifndef Z3D_CORE___ZCOREPLUGIN_H
#define Z3D_CORE___ZCOREPLUGIN_H

#include "zcore_global.h"

#include <QObject>

namespace Z3D
{

class Z3D_CORE_SHARED_EXPORT ZCorePlugin : public QObject
{
    Q_OBJECT
public:
    explicit ZCorePlugin(QObject *parent = 0);
    virtual ~ZCorePlugin() {}

    /// plugin information
    virtual QString id() = 0;
    virtual QString name() = 0;
    virtual QString version() = 0;

signals:

public slots:
};

} // namespace Z3D

#endif // Z3D_CORE___ZCOREPLUGIN_H
