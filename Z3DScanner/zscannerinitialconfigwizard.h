#ifndef ZSCANNERINITIALCONFIGWIZARD_H
#define ZSCANNERINITIALCONFIGWIZARD_H

#include <QWidget>

namespace Ui {
class ZScannerInitialConfigWizard;
}

class ZScannerInitialConfigWizard : public QWidget
{
    Q_OBJECT

public:
    explicit ZScannerInitialConfigWizard(QWidget *parent = 0);
    ~ZScannerInitialConfigWizard();

private slots:
    void on_pushButton_clicked();

private:
    Ui::ZScannerInitialConfigWizard *ui;
};

#endif // ZSCANNERINITIALCONFIGWIZARD_H
