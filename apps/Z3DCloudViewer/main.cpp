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

#include "zpointcloudviewercontroller.h"

#include "ZGui/zapplication.h"
#include "ZPointCloud/zpointcloudprovider.h"

#include <QtQml/QQmlApplicationEngine>

int main(int argc, char *argv[])
{
    /// to print out diagnostic information about each plugin it (Qt) tries to load
    //qputenv("QT_DEBUG_PLUGINS", "1");
    //qputenv("QSG_INFO", "1");
    qputenv("QT3D_RENDERER", "rhi"); // should be the default, but set anyways

    Z3D::ZApplication app(argc, argv);
    app.loadPlugins();
    Z3D::ZPointCloudProvider::loadPlugins();

    QQmlApplicationEngine engine;

    //! TODO: This is ugly, find a better way to do this
    engine.addImportPath(":/z3d.neuvision/");

    //! FIXME create Qml module to use auto registration / qt_add_qml_module
    qmlRegisterType<Z3D::ZPointCloudViewerController>("Z3D.ZPointCloudViewer", 1, 0, "ZPointCloudViewerController");

    engine.load(QUrl(QStringLiteral("qrc:/qml/main.qml")));

    return app.exec();
}
