#include <QApplication>

#include "zcloudview.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    Z3D::ZCloudView w;
    w.show();

    return a.exec();
}
