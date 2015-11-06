#include "zlasertriangulationcalibratorwindow.h"
#include "ui_zlasertriangulationcalibratorwindow.h"

#include "zcalibrationpatternfinderprovider.h"
#include "zlasertriangulationcalibrator.h"
#include "zltscalibrationimagemodel.h"

#include <QFileDialog>
#include <QProgressBar>

namespace Z3D
{

ZLaserTriangulationCalibratorWindow::ZLaserTriangulationCalibratorWindow(ZCalibratedCamera::WeakPtr camera3D, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ZLaserTriangulationCalibratorWindow),
    m_camera(camera3D),
    m_model(new ZLTSCalibrationImageModel(this)),
    m_currentPatternFinder(0),
    m_ltsCalibrator(new ZLaserTriangulationCalibrator())
{
    ui->setupUi(this);

    /// create and add progress bar used in the statusbar
    m_statusProgressBar = new QProgressBar();
    m_statusProgressBar->setVisible(false);
    m_statusProgressBar->setMaximumWidth(300);
    ui->statusbar->addPermanentWidget(m_statusProgressBar);

    /// always start with welcome page visible
    ui->stackedWidget->setCurrentIndex(0);

    /// add 3d view
    ui->viewerLayout->addWidget(m_ltsCalibrator->getViewerWidget());

    /// configure lts calibrator
    m_ltsCalibrator->setCameraCalibration(m_camera->calibration().data());
    m_ltsCalibrator->setImageModel(m_model);
    QObject::connect(m_ltsCalibrator, SIGNAL(progressChanged(float,QString)),
                     this, SLOT(onProgressChanged(float,QString)));
    QObject::connect(m_ltsCalibrator, SIGNAL(calibrationChanged()),
                     this, SLOT(onCalibrationChanged()));

    /// to update current pattern finder
    QObject::connect(ui->calibrationPatternTypeComboBox, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(onCalibrationPatternTypeChanged(int)));

    /// load different calibration pattern finder types
    m_patternFinderList = ZCalibrationPatternFinderProvider::getAll();

    foreach (ZCalibrationPatternFinder::Ptr patternFinder, m_patternFinderList)
        ui->calibrationPatternTypeComboBox->addItem(patternFinder->name());

    /// configure image list view
    ui->listView->setModel(m_model);
    QObject::connect(ui->listView->selectionModel(), SIGNAL(currentChanged(QModelIndex,QModelIndex)),
                     this, SLOT(onCurrentSelectionChanged(QModelIndex,QModelIndex)));
    /// whenever someone clicks the list view, switch to image view
    QObject::connect(ui->listView, SIGNAL(clicked(QModelIndex)),
                     this, SLOT(on_imageViewPageButton_clicked()));

    /// set up image view type
    ui->imageViewTypeComboBox->addItem(tr("View calibration pattern"));
    ui->imageViewTypeComboBox->addItem(tr("View original"));

    /// move to worker thread
    QThread *m_workerThread = new QThread(this);
    m_ltsCalibrator->moveToThread(m_workerThread);
    foreach (ZCalibrationPatternFinder::Ptr patternFinder, m_patternFinderList)
        patternFinder->moveToThread(m_workerThread);
    m_workerThread->start(QThread::HighPriority);
}

ZLaserTriangulationCalibratorWindow::~ZLaserTriangulationCalibratorWindow()
{
    delete ui;
}

void ZLaserTriangulationCalibratorWindow::closeEvent(QCloseEvent * /*event*/)
{
    deleteLater();
}

void ZLaserTriangulationCalibratorWindow::onCurrentSelectionChanged(QModelIndex current, QModelIndex previous)
{
    Q_UNUSED(previous)

    if (!current.isValid())
        return;

    QVariant variant = current.model()->data(current, ZLTSCalibrationImageModel::DataRole);
    if (!variant.isValid() || !variant.canConvert<Z3D::ZLTSCalibrationImage::Ptr>())
        return;

    Z3D::ZLTSCalibrationImage::Ptr image = variant.value<Z3D::ZLTSCalibrationImage::Ptr>();

    if (image) {
        if (ui->imageViewTypeComboBox->currentIndex())
            ui->label->setPixmap(image->pixmapFromUrl(Z3D::ZLTSCalibrationImage::OriginalImage));
        else
            ui->label->setPixmap(image->pixmapFromUrl(Z3D::ZLTSCalibrationImage::PatternImage));
    }

    ui->imageViewPageButton->setChecked(true);
}

void ZLaserTriangulationCalibratorWindow::onCalibrationPatternTypeChanged(int index)
{
    /// remove previous config widget and hide it
    if (m_currentPatternFinder) {
        QWidget *previousWidget = m_currentPatternFinder->configWidget();
        previousWidget->setVisible(false);
        ui->calibrationPatternConfigLayout->removeWidget(previousWidget);
    }

    /// update current pattern finder
    m_currentPatternFinder = m_patternFinderList[index];
    m_ltsCalibrator->setPatternFinder(m_currentPatternFinder);

    /// add config widget
    QWidget *currentWidget = m_currentPatternFinder->configWidget();
    currentWidget->setVisible(true);
    ui->calibrationPatternConfigLayout->addWidget(currentWidget);
}

void ZLaserTriangulationCalibratorWindow::onProgressChanged(float progress, QString message)
{
    m_statusProgressBar->setValue(100 * progress);
    m_statusProgressBar->setVisible(progress < 1.);

    if (progress == 1 && message.isEmpty())
        ui->statusbar->showMessage(tr(""), 5000);
    else if (!message.isEmpty())
        ui->statusbar->showMessage(message, progress < 1 ? 0 : 5000);
}

void ZLaserTriangulationCalibratorWindow::onCalibrationChanged()
{
    /// enable button again, calibration has finished
    ui->runCalibrationButton->setEnabled(true);
}

void ZLaserTriangulationCalibratorWindow::on_pushButton_clicked()
{
    QString dirname = QFileDialog::getExistingDirectory(this, "Select image folder", "", QFileDialog::ReadOnly);

    if (dirname.isEmpty()) {
        /// nothing selected
        return;
    }

    m_model->addFolder(dirname);
}

void ZLaserTriangulationCalibratorWindow::on_imageViewPageButton_clicked()
{
    ui->imageViewPageButton->setChecked(true);
    ui->stackedWidget->setCurrentIndex(1);
}

void ZLaserTriangulationCalibratorWindow::on_cameraViewPageButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(2);
}

void ZLaserTriangulationCalibratorWindow::on_calibrationPageButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(3);
}

void ZLaserTriangulationCalibratorWindow::on_runCalibrationButton_clicked()
{
    /// disable button until calibration has finished
    ui->runCalibrationButton->setEnabled(false);

    //m_ltsCalibrator->findCalibrationPattern();
    m_ltsCalibrator->calibrate();
}

void ZLaserTriangulationCalibratorWindow::on_imageViewTypeComboBox_currentIndexChanged(int /*index*/)
{
    onCurrentSelectionChanged(ui->listView->selectionModel()->currentIndex(), QModelIndex());
}

void ZLaserTriangulationCalibratorWindow::on_calibrationPatternViewButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}

} // namespace Z3D
