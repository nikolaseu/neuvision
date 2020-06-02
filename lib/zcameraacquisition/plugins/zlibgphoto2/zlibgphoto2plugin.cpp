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

#include "zlibgphoto2plugin.h"

#include "zlibgphoto2camera.h"

#include "ZCameraAcquisition/zcamerainfo.h"
#include "ZCore/zlogging.h"

#include <gphoto2/gphoto2-camera.h>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcameraacquisition.zlibgphoto2", QtInfoMsg)

namespace Z3D
{

static GPPortInfoList  *portinfolist = nullptr;
static CameraAbilitiesList *abilities = nullptr;

/*
 * This detects all currently attached cameras and returns
 * them in a list. It avoids the generic usb: entry.
 *
 * This function does not open nor initialize the cameras yet.
 */
int sample_autodetect(CameraList *list, GPContext *context) {
    gp_list_reset (list);
    return gp_camera_autodetect (list, context);
}

/*
 * This function opens a camera depending on the specified model and port.
 */
int sample_open_camera(Camera ** camera, const char *model, const char *port, GPContext *context) {
    int  ret, m, p;
    CameraAbilities a;
    GPPortInfo pi;

    ret = gp_camera_new (camera);
    if (ret < GP_OK) return ret;

    if (!abilities) {
        /* Load all the camera drivers we have... */
        ret = gp_abilities_list_new (&abilities);
        if (ret < GP_OK) return ret;

        ret = gp_abilities_list_load (abilities, context);
        if (ret < GP_OK) return ret;
    }

    /* First lookup the model / driver */
    m = gp_abilities_list_lookup_model (abilities, model);
    if (m < GP_OK) return ret;

    ret = gp_abilities_list_get_abilities (abilities, m, &a);
    if (ret < GP_OK) return ret;

    ret = gp_camera_set_abilities (*camera, a);
    if (ret < GP_OK) return ret;

    if (!portinfolist) {
        /* Load all the port drivers we have... */
        ret = gp_port_info_list_new (&portinfolist);
        if (ret < GP_OK) return ret;

        ret = gp_port_info_list_load (portinfolist);
        if (ret < 0) return ret;

        ret = gp_port_info_list_count (portinfolist);
        if (ret < 0) return ret;
    }

    /* Then associate the camera with the specified port */
    p = gp_port_info_list_lookup_path (portinfolist, port);
    switch (p) {
    case GP_ERROR_UNKNOWN_PORT:
        zWarning() << "The port you specified (" << port << "can not be found."
                   << "Please specify one of the ports found by 'gphoto2 --list-ports'"
                   << "and make sure the spelling is correct (i.e. with prefix 'serial:' or 'usb:').";
        break;
    default:
        break;
    }
    if (p < GP_OK) return p;

    ret = gp_port_info_list_get_info (portinfolist, p, &pi);
    if (ret < GP_OK) return ret;

    ret = gp_camera_set_port_info (*camera, pi);
    if (ret < GP_OK) return ret;

    return GP_OK;
}

static void ctx_error_func (GPContext * /*context*/, const char *str, void * /*data*/)
{
    zWarning() << str;
}

static void ctx_status_func (GPContext * /*context*/, const char *str, void * /*data*/)
{
    zDebug() << str;
}






ZLibGPhoto2Plugin::ZLibGPhoto2Plugin()
{
    /* This is the mandatory part */
    context = gp_context_new();

    /* All the parts below are optional! */
    gp_context_set_error_func (context, ctx_error_func, nullptr);
    gp_context_set_status_func (context, ctx_status_func, nullptr);

    /* also:
    gp_context_set_cancel_func    (p->context, ctx_cancel_func,  p);
    gp_context_set_message_func   (p->context, ctx_message_func, p);
    if (isatty (STDOUT_FILENO))
        gp_context_set_progress_funcs (p->context,
            ctx_progress_start_func, ctx_progress_update_func,
            ctx_progress_stop_func, p);
    */
}

ZLibGPhoto2Plugin::~ZLibGPhoto2Plugin()
{

}

QString ZLibGPhoto2Plugin::displayName() const
{
    return QString("gPhoto2");
}

QList<ZCameraInfo *> ZLibGPhoto2Plugin::getConnectedCameras()
{
    QList<ZCameraInfo*> camerasList;

    /* Detect all the cameras that can be autodetected... */
    CameraList *list;
    int ret = gp_list_new(&list);
    if (ret < GP_OK) {
        zWarning() << "Error trying to create CameraList" << ret;
        return camerasList;
    }

    int count = sample_autodetect(list, context);
    if (count < GP_OK) {
        zWarning() << "Error trying to detect cameras" << count;
        return camerasList;
    }

    zDebug() << "found" << count << "cameras.";
    for (int i = 0; i < count; i++) {
        Camera *cam;
        const char *name, *value;
        gp_list_get_name(list, i, &name);
        gp_list_get_value(list, i, &value);
        ret = sample_open_camera(&cam, name, value, context);
        if (ret < GP_OK) {
            zWarning() << "Camera:" << name << "on port:" << value << "failed to open";
            continue;
        }

        /* Now call a simple function in each of those cameras. */
        CameraText text;
        ret = gp_camera_get_summary(cam, &text, context);
        if (ret < GP_OK) {
            zWarning() << "Failed to get summary";
            continue;
        }

        zDebug() << "Found camera:" << name << "on port:" << value << "\n"
                 << "Summary:\n"
                 << text.text;

        QVariantMap extraData;
        extraData["description"] = QString(text.text);
        extraData["model"] = QString(name);
        extraData["port"] = QString(value);

        camerasList << new ZCameraInfo(this, QString(name), extraData);

        /// release camera
        gp_camera_exit(cam, context);
        gp_camera_free(cam);
    }

    return camerasList;
}

ZCameraPtr ZLibGPhoto2Plugin::getCamera(QVariantMap options)
{
    QString model = options.value("model").toString();
    QString port = options.value("port").toString();

    Camera *cam;
    int ret = sample_open_camera(&cam, qPrintable(model), qPrintable(port), context);
    if (ret < GP_OK) {
        zWarning() << "Camera:" << model << "on port:" << port << "failed to open";
        return nullptr;
    }

    return ZCameraPtr(new ZLibGPhoto2Camera(context, cam));
}

} // namespace Z3D
