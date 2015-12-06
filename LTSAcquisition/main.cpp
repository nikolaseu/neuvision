#include "mainwindow.h"
#include "zapplication.h"


int main(int argc, char* argv[])
{
    Z3D::ZApplication app(argc,argv);

    MainWindow window;
    window.show();

    return app.exec();
}
