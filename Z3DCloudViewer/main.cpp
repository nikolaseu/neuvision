#include "zapplication.h"
#include "zcloudviewwindow.h"

int main(int argc, char *argv[])
{
    Z3D::ZApplication a(argc, argv);

    Z3D::ZCloudViewWindow w;
    w.show();

    return a.exec();
}
