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

#include "zlibgphoto2camera.h"

#include "ZCameraAcquisition/zcameraimage.h"

#include <QDebug>
#include <QImage> //! FIXME we can replace QImage with cv::Mat to remove dependency in Qt5::Gui
#include <QSize>
#include <QThread>
#include <QTimer>


namespace Z3D
{

static size_t my_strftime(char *s, size_t max, const char *fmt, const struct tm *tm)
{
    return strftime(s, max, fmt, tm);
}

static int print_widget (GPContext *context, const char *name, CameraWidget *widget, QList<ZCameraInterface::ZCameraAttribute> &attributeList) {
    const char *label;
    CameraWidgetType type;
    int ret, readonly;

    ret = gp_widget_get_type (widget, &type);
    if (ret != GP_OK)
        return ret;

    ret = gp_widget_get_label (widget, &label);
    if (ret != GP_OK)
        return ret;

    ret = gp_widget_get_readonly (widget, &readonly);
    if (ret != GP_OK)
        return ret;

    printf ("Label: %s\n", label); /* "Label:" is not i18ned, the "label" variable is */
    printf ("Readonly: %d\n", readonly);

    QString nameStr = QString(name);
    QString labelStr = QString(label);
    ZCameraInterface::ZCameraAttribute attr;
    attr.id = nameStr;
    attr.path = (nameStr.startsWith("/") ? nameStr.mid(1) : nameStr).replace("/", "::");
    attr.label = labelStr;
    attr.description = labelStr;
    attr.readable = true;
    attr.writable = !readonly;

    switch (type) {
    case GP_WIDGET_TEXT: {  /* char *  */
        char *txt;

        ret = gp_widget_get_value (widget, &txt);
        if (ret == GP_OK) {
            printf ("Type: TEXT\n"); /* parsed by scripts, no i18n */
            printf ("Current: %s\n",txt);

            attr.type = ZCameraInterface::CameraAttributeTypeString;
            attr.value = QString(txt);
        } else {
            gp_context_error (context, "Failed to retrieve value of text widget %s.", name);
        }
        break;
    }
    case GP_WIDGET_RANGE: { /* float  */
        float f, t,b,s;

        ret = gp_widget_get_range (widget, &b, &t, &s);
        if (ret == GP_OK)
            ret = gp_widget_get_value (widget, &f);
        if (ret == GP_OK) {
            printf ("Type: RANGE\n"); /* parsed by scripts, no i18n */
            printf ("Current: %g\n", f); /* parsed by scripts, no i18n */
            printf ("Bottom: %g\n", b); /* parsed by scripts, no i18n */
            printf ("Top: %g\n", t); /* parsed by scripts, no i18n */
            printf ("Step: %g\n", s); /* parsed by scripts, no i18n */
        } else {
            gp_context_error (context, "Failed to retrieve values of range widget %s.", name);
        }
        break;
    }
    case GP_WIDGET_TOGGLE: { /* int  */
        int t;

        ret = gp_widget_get_value (widget, &t);
        if (ret == GP_OK) {
            printf ("Type: TOGGLE\n");
            printf ("Current: %d\n",t);

            attr.type = ZCameraInterface::CameraAttributeTypeBool;
            attr.value = bool(t);
        } else {
            gp_context_error (context, "Failed to retrieve values of toggle widget %s.", name);
        }
        break;
    }
    case GP_WIDGET_DATE:  {  /* int   */
        int t;
        time_t xtime;
        struct tm *xtm;
        char timebuf[200];

        ret = gp_widget_get_value (widget, &t);
        if (ret != GP_OK) {
            gp_context_error (context, "Failed to retrieve values of date/time widget %s.", name);
            break;
        }
        xtime = t;
        xtm = localtime (&xtime);
        ret = my_strftime (timebuf, sizeof(timebuf), "%c", xtm);
        printf ("Type: DATE\n");
        printf ("Current: %d\n", t);
        printf ("Printable: %s\n", timebuf);
        printf ("Help: %s\n", "Use 'now' as the current time when setting.\n");

        attr.type = ZCameraInterface::CameraAttributeTypeString;
        attr.value = QString(timebuf);

        break;
    }
    case GP_WIDGET_MENU:
    case GP_WIDGET_RADIO: { /* char *  */
        int cnt, i;
        char *current;

        ret = gp_widget_get_value (widget, &current);
        if (ret == GP_OK) {
            attr.type = ZCameraInterface::CameraAttributeTypeEnum;

            cnt = gp_widget_count_choices (widget);
            if (type == GP_WIDGET_MENU)
                printf ("Type: MENU\n");
            else
                printf ("Type: RADIO\n");
            printf ("Current: %s\n",current);
            for ( i=0; i<cnt; i++) {
                const char *choice;
                ret = gp_widget_get_choice (widget, i, &choice);
                printf ("Choice: %d %s\n", i, choice);

                attr.enumNames << QString(choice);
                if (strcmp(current, choice) == 0) {
                    attr.enumValue = i;
                }
            }
        } else {
            gp_context_error (context, "Failed to retrieve values of radio widget %s.", name);
        }
        break;
    }

    /* ignore: */
    case GP_WIDGET_WINDOW:
    case GP_WIDGET_SECTION:
    case GP_WIDGET_BUTTON:
        break;
    }

    attributeList << attr;

    printf ("END\n");
    return GP_OK;
}

static void display_widgets (Camera *camera, GPContext *context, CameraWidget *widget, char *prefix, QList<ZCameraInterface::ZCameraAttribute> &attributeList)
{
    int  ret, n, i;
    char *newprefix;
    const char *label, *name, *uselabel;
    CameraWidgetType type;

    gp_widget_get_label (widget, &label);
    /* fprintf(stderr,"label is %s\n", label); */
    ret = gp_widget_get_name (widget, &name);
    /* fprintf(stderr,"name is %s\n", name); */
    gp_widget_get_type (widget, &type);

    if (strlen(name)) {
        uselabel = name;
    } else {
        uselabel = label;
    }

    n = gp_widget_count_children (widget);

    newprefix = (char *) malloc(strlen(prefix)+1+strlen(uselabel)+1);
    if (!newprefix) {
        abort();
    }

    sprintf(newprefix,"%s/%s",prefix,uselabel);

    if ((type != GP_WIDGET_WINDOW) && (type != GP_WIDGET_SECTION)) {
        printf("%s\n",newprefix);
        print_widget (context, newprefix, widget, attributeList);
    }

    for (i=0; i<n; i++) {
        CameraWidget *child;
        ret = gp_widget_get_child (widget, i, &child);
        if (ret != GP_OK) {
            continue;
        }

        display_widgets (camera, context, child, newprefix, attributeList);
    }

    free(newprefix);
}

int list_all_config_action (Camera *camera, GPContext *context, QList<ZCameraInterface::ZCameraAttribute> &attributeList)
{
    CameraWidget *rootconfig;

    int ret = gp_camera_get_config(camera, &rootconfig, context);
    if (ret != GP_OK) {
        return ret;
    }

    display_widgets (camera, context, rootconfig, "", attributeList);
    gp_widget_free (rootconfig);

    return GP_OK;
}

/*
 * This function looks up a label or key entry of
 * a configuration widget.
 * The functions descend recursively, so you can just
 * specify the last component.
 */
static int _lookup_widget(CameraWidget *widget, const char *key, CameraWidget **child) {
    int ret = gp_widget_get_child_by_name (widget, key, child);
    if (ret < GP_OK) {
        ret = gp_widget_get_child_by_label (widget, key, child);
    }

    return ret;
}

/* Sets a string configuration value.
 * This can set for:
 *  - A Text widget
 *  - The current selection of a Radio Button choice
 *  - The current selection of a Menu choice
 *
 * Sample (for Canons eg):
 *   get_config_value_string (camera, "owner", &ownerstr, context);
 */
int set_config_value_string(Camera *camera, const char *key, const char *val, GPContext *context) {
    CameraWidget *widget = nullptr, *child = nullptr;
    CameraWidgetType type;
    int ret;

    ret = gp_camera_get_config (camera, &widget, context);
    if (ret < GP_OK) {
        fprintf (stderr, "camera_get_config failed: %d\n", ret);
        return ret;
    }

    ret = _lookup_widget (widget, key, &child);
    if (ret < GP_OK) {
        fprintf (stderr, "lookup widget failed: %d\n", ret);
        goto out;
    }

    /* This type check is optional, if you know what type the label
    * has already. If you are not sure, better check. */
    ret = gp_widget_get_type (child, &type);
    if (ret < GP_OK) {
        fprintf (stderr, "widget get type failed: %d\n", ret);
        goto out;
    }

    switch (type) {
    case GP_WIDGET_MENU:
    case GP_WIDGET_RADIO:
    case GP_WIDGET_TEXT:
        break;
    default:
        fprintf (stderr, "widget has bad type %d\n", type);
        ret = GP_ERROR_BAD_PARAMETERS;
        goto out;
    }

    /* This is the actual set call. Note that we keep
    * ownership of the string and have to free it if necessary.
    */
    ret = gp_widget_set_value (child, val);
    if (ret < GP_OK) {
        fprintf (stderr, "could not set widget value: %d\n", ret);
        goto out;
    }

    ret = gp_camera_set_single_config (camera, key, child, context);
    if (ret != GP_OK) {
        /* This stores it on the camera again */
        ret = gp_camera_set_config (camera, widget, context);
        if (ret < GP_OK) {
            fprintf (stderr, "camera_set_config failed: %d\n", ret);
            return ret;
        }
    }

out: /// FIXME remove goto calls
    gp_widget_free (widget);
    return ret;
}

static void capture_to_memory(Camera *camera, GPContext *context, const char **ptr, unsigned long int *size) {
    int retval;
    CameraFile *file;
    CameraFilePath camera_file_path;

    printf("Capturing.\n");

    /* NOP: This gets overridden in the library to /capt0000.jpg */
    strcpy(camera_file_path.folder, "/");
    strcpy(camera_file_path.name, "foo.jpg");

    retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
    printf("  Retval: %d\n", retval);

    printf("Pathname on the camera: %s/%s\n", camera_file_path.folder, camera_file_path.name);

    retval = gp_file_new(&file);
    printf("  Retval: %d\n", retval);
    retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name, GP_FILE_TYPE_NORMAL, file, context);
    printf("  Retval: %d\n", retval);

    gp_file_get_data_and_size (file, ptr, size);

    printf("Deleting.\n");
    retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name, context);

    printf("  Retval: %d\n", retval);
    /*gp_file_free(file); */
}

QImage captureImage(Camera *camera, GPContext *context) {
    const unsigned char *buffer;
    unsigned long size;
    capture_to_memory(camera, context, (const char **)&buffer, &size);

    if (!size) {
        return QImage();
    }

    return QImage::fromData(buffer, int(size), "jpg")
            .convertToFormat(QImage::Format_Grayscale8);
}










ZLibGPhoto2Camera::ZLibGPhoto2Camera(GPContext *context, Camera *camera, QObject *parent)
    : ZCameraBase(parent)
    , context(context)
    , cam(camera)
{
    setBufferSize(5);

    /// create a thread for the camera and move the camera to it
    QThread *cameraThread = new QThread();
    qDebug() << qPrintable(
                    QString("[%1] moving camera to its own thread (0x%2)")
                    .arg(this->uuid())
                    .arg((long)cameraThread, 0, 16));
    this->moveToThread(cameraThread);
    /// start thread with the highest priority
    cameraThread->start(QThread::TimeCriticalPriority);
}

ZLibGPhoto2Camera::~ZLibGPhoto2Camera()
{
    /// release camera
    gp_camera_exit(cam, context);
    gp_camera_free(cam);
}

bool ZLibGPhoto2Camera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition()) {
        return false;
    }

    /// start running grab loop in the camera's thread
    m_stopThreadRequested = false;
    QTimer::singleShot(0, this, &ZLibGPhoto2Camera::grabLoop);

    return true;
}

bool ZLibGPhoto2Camera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition()) {
        return false;
    }

    m_stopThreadRequested = true;

    return true;
}

ZCameraImagePtr ZLibGPhoto2Camera::getSnapshot()
{
    QImage picture = captureImage(cam, context);
    if (picture.isNull()) {
        return nullptr;
    }

    ZCameraImagePtr currentImage = getNextBufferImage(picture.width(), picture.height(), 0, 0, 1);

    /// copy data
    currentImage->setBuffer(picture.bits());

    return currentImage;
}

QList<ZCameraInterface::ZCameraAttribute> ZLibGPhoto2Camera::getAllAttributes()
{
    QList<ZCameraInterface::ZCameraAttribute> attributeList;

    list_all_config_action(cam, context, attributeList);

    return attributeList;
}

QVariant ZLibGPhoto2Camera::getAttribute(const QString &/*id*/) const
{
    //! TODO
    return QVariant("INVALID");
}

bool ZLibGPhoto2Camera::setAttribute(const QString &id, const QVariant &value, bool notify)
{
    qDebug() << "trying to set" << id << "to" << value;

    QString propName = id.split("/").last();

    int ret = set_config_value_string(cam, qPrintable(propName), qPrintable(value.toString()), context);
    if (ret != GP_OK) {
        qWarning() << "failed setting single config" << id << ret;
        return false;
    }

    /// now the mutex is released, we could update settings in case something changed
    if (notify) {
        emit attributeChanged("","");
    }

    return true;
}

void ZLibGPhoto2Camera::grabLoop()
{
    QImage picture;
    int frameCounter = -1;

    while (!m_stopThreadRequested) {
        {
            /// thread safe
            QMutexLocker locker(&m_mutex);

            picture = captureImage(cam, context);
            if (picture.isNull()) {
                continue;
            }
        }

        ZCameraImagePtr currentImage = getNextBufferImage(picture.width(), picture.height(), 0, 0, 1);

        /// copy data
        currentImage->setBuffer(picture.bits());

        /// set image number
        currentImage->setNumber(++frameCounter);

        //qDebug() << "frame" << frameCounter;

        if (!m_stopThreadRequested) {
            /// notify
            emit newImageReceived(currentImage);
        }
    }

    qDebug() << "acquired" << frameCounter << "images";

    if (!m_stopThreadRequested) {
        /// if we stopped because of some error, notify
        stopAcquisition();
    }
}

} // namespace Z3D
