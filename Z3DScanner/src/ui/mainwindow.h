#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "zsimplepointcloud.h"

#include <QMainWindow>
#include <QPointer>

#include <opencv2/core/core.hpp>

namespace Ui {
class MainWindow;
}

namespace Z3D {
class ZPatternProjection;
class ZStructuredLightSystem;
}

class ZPointCloudWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

signals:

private slots:
    void init();

    virtual void closeEvent(QCloseEvent *event);

    //
    void onPatternProjectionTypeChanged(int index);
    void onStructuredLightSystemTypeChanged(int index);

    void onScanFinished(Z3D::ZSimplePointCloud::Ptr cloud);

private:
    Ui::MainWindow *ui;

    QList<Z3D::ZPatternProjection *> m_patternProjectionList;
    Z3D::ZPatternProjection *m_currentPatternProjection;

    QList<Z3D::ZStructuredLightSystem *> m_structuredLightList;
    Z3D::ZStructuredLightSystem *m_currentStructuredLightSystem;

    ZPointCloudWidget *m_pointCloudWidget;
};

#endif // MAINWINDOW_H
