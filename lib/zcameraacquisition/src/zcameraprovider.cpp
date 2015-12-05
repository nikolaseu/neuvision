#include "zcameraprovider.h"

#include "zcamerainterface.h"
#include "zcameraplugininterface.h"

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QPluginLoader>
#include <QThread>

namespace Z3D
{

QMap< QString, ZCameraPluginInterface *> ZCameraProvider::m_plugins;
Z3D::ZCameraListModel ZCameraProvider::m_model;

void ZCameraProvider::loadPlugins(QString folder)
{
    QDir pluginsDir = QDir(folder);

    /// if no folder is indicated, use current running folder and standard search path
    if (folder.isEmpty()) {
        pluginsDir = QCoreApplication::applicationDirPath();

    #if defined(Q_OS_WIN)
//        if (pluginsDir.dirName().toLower() == "debug" || pluginsDir.dirName().toLower() == "release")
//            pluginsDir.cdUp();
    #elif defined(Q_OS_MAC)
        if (pluginsDir.dirName() == "MacOS") {
            pluginsDir.cdUp();
            pluginsDir.cdUp();
            pluginsDir.cdUp();
        }
    #endif

        if (!(pluginsDir.cd("plugins") && pluginsDir.cd("cameraacquisition"))) {
            qWarning() << "standard camera acquisition plugin folder 'plugins/cameraacquisition' not found in" << pluginsDir.absolutePath();
            return;
        }
    }

    qDebug() << "searching for camera acquisition plugins in" << pluginsDir.absolutePath();

    QStringList filters;
    filters << "*.dll" << "*.so" << "*.dylib";
    pluginsDir.setNameFilters(filters);

    foreach (QString fileName, pluginsDir.entryList(QDir::Files)) {
        QPluginLoader loader(pluginsDir.absoluteFilePath(fileName));
        QObject *pluginInstance = loader.instance();
        if (pluginInstance) {
            ZCameraPluginInterface *plugin = qobject_cast<ZCameraPluginInterface *>(pluginInstance);
            if (plugin) {
                qDebug() << "camera plugin loaded. type:" << plugin->id()
                         << "version:" << plugin->version()
                         << "filename:" << fileName;


//                plugin->getConnectedCameras();

//                    QObject::connect(cameraPlugin, SIGNAL(cameraConnected(QString)),
//                                     this, SLOT(onCameraConnected(QString)));

                m_plugins.insert(plugin->id(), plugin);
            } else {
                qWarning() << "invalid camera plugin:" << fileName;
            }
        } else {
            qWarning() << "error loading camera plugin" << fileName << "->" << loader.errorString();
        }
    }
}

void ZCameraProvider::unloadPlugins()
{
    foreach(ZCameraPluginInterface *plugin, m_plugins.values())
        delete plugin;

    m_plugins.clear();
}

ZCameraInterface::Ptr ZCameraProvider::getCamera(QString pluginName, QVariantMap options)
{
    ZCameraInterface::Ptr camera;

    if (m_plugins.contains(pluginName)) {
        camera = m_plugins[pluginName]->getCamera(options);

        if (camera) {
            /// load configuration file
            if (options.contains("ConfigFile")) {
                QString configFileName = options.value("ConfigFile").toString();
                camera->loadConfiguration(configFileName);
            }

            /// initial preset to load ("BaseAcquisitionMode" is always loaded)
            if (options.contains("Preset")) {
                QString presetName = options.value("Preset").toString();
                camera->loadPresetConfiguration(presetName);
            }

            /// custom buffer size
            if (options.contains("BufferSize")) {
                int bufferSize = options.value("BufferSize").toInt();
                if (camera->setBufferSize(bufferSize))
                    qDebug() << "camera" << camera->uuid() << "using a buffer size of" << bufferSize;
            }
/* TODO: each camera will have to do this...
            /// create a thread for the camera and move the camera to it
            if (QThread::currentThread() == camera->thread()) {
                QThread *cameraThread = new QThread();
                qDebug() << qPrintable(
                                QString("[%1] moving camera to its own thread (0x%2)")
                                .arg(camera->uuid())
                                .arg((long)cameraThread, 0, 16));
                camera->moveToThread(cameraThread);
                /// start thread with the highest priority
                cameraThread->start(QThread::TimeCriticalPriority);
            } else {
                qWarning() << qPrintable(
                                  QString("[%1] camera thread (0x%2) is not the current thread (0x%3). Cannot move camera to its own thread!")
                                  .arg(camera->uuid())
                                  .arg((long)camera->thread(), 0, 16)
                                  .arg((long)QThread::currentThread(), 0, 16));
            }
*/
            m_model.add(camera.data());
        }
    } else {
        qWarning() << "camera plugin not found:" << pluginName;
    }

    return camera;
}

ZCameraInterface::Ptr ZCameraProvider::getCamera(QSettings *settings)
{
    QVariantMap options;
    foreach(QString key, settings->allKeys()) {
        options[key] = settings->value(key);
    }

    QString deviceType = options["Type"].toString();

    return getCamera(deviceType, options);
}

QList<ZCameraInterface::Ptr> ZCameraProvider::loadCameras(QString folder)
{
    /// if no folder is indicated, use current running folder + "/cameras"
    if (folder.isEmpty()) {
        folder = QDir::current().absoluteFilePath("cameras");
    }

    QDir configDir(folder);

    qDebug() << "loading cameras from" << configDir.absolutePath();

    QList<Z3D::ZCameraInterface::Ptr> cameraList;

    QStringList filters;
    filters << "*.ini" << "*.json"; //! TODO: agregar para leer configuracion en JSON
    configDir.setNameFilters(filters);

    foreach (QString fileName, configDir.entryList(QDir::Files)) {
        qDebug() << "found" << fileName;
        QSettings settings(configDir.absoluteFilePath(fileName), QSettings::IniFormat);
        settings.beginGroup("Camera");
        cameraList << getCamera(&settings);
    }

    return cameraList;
}

ZCameraListModel *ZCameraProvider::model()
{
    return &m_model;
}

QList<ZCameraPluginInterface *> ZCameraProvider::availablePlugins()
{
    QList<ZCameraPluginInterface *> pluginList;

    foreach(ZCameraPluginInterface *plugin, m_plugins.values())
        pluginList << plugin;

    return pluginList;
}


} // namespace Z3D
