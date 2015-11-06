#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "zcamerainterface.h"

#include <QMainWindow>

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void onCameraSelected(Z3D::ZCameraInterface::Ptr camera);

    void on_continueButton_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
