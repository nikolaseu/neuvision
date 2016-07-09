#include "zapplication.h"

#include "zloghandler_p.h"
#include "zpluginloader.h"

#include <QDebug>
#include <QSplashScreen>
#include <QThreadPool>

namespace Z3D
{

ZApplication::ZApplication(int &argc, char **argv)
    : QApplication(argc, argv)
    , m_splash(nullptr)
{
    /// set message handler for debug
#if !defined(Q_OS_ANDROID)
#if QT_VERSION < 0x050000
    qInstallMsgHandler(&ZLogHandler);
#else
    qInstallMessageHandler(&ZLogHandler);
#endif
#endif

#ifdef Z3D_THREAD_COUNT_LIMIT
    QThreadPool::globalInstance()->setMaxThreadCount(Z3D_THREAD_COUNT_LIMIT);
#endif
}

void ZApplication::loadPlugins()
{
    ZPluginLoader::instance()->loadPlugins();
}

QSplashScreen * ZApplication::showSplashScreen()
{
    if (!m_splash) {
        QPixmap pixmap(":/splash");
        m_splash = new QSplashScreen(pixmap);
        m_splash->show();

        connect(ZPluginLoader::instance(), &ZPluginLoader::progressChanged,
                [&](float progress, QString message) {
                    qDebug() << "progress:" << progress << "message:" << message;
                    m_splash->showMessage(message);
                    processEvents();
                }
        );
    }

    return m_splash;
}

void ZApplication::finishSplashScreen(QWidget *widget)
{
    if (m_splash) {
        m_splash->finish(widget);
    }
}

} // namespace Z3D
