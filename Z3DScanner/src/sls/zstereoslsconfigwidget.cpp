#include "zstereoslsconfigwidget.h"
#include "ui_zstereoslsconfigwidget.h"

#include "zcameracalibratorwidget.h"
#include "zcamerapreviewer.h"
#include "zstereosls.h"

#include <QMenu>

namespace Z3D {

ZStereoSLSConfigWidget::ZStereoSLSConfigWidget(ZStereoSLS *stereoSLS, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZStereoSLSConfigWidget)
    , m_stereoSLS(stereoSLS)
{
    ui->setupUi(this);

    connect(ui->calibrateSystemButton, &QPushButton::clicked,
            [&](){ m_stereoSLS->getCalibrationWindow()->show(); });

    connect(m_stereoSLS, &ZStereoSLS::maxValidDistanceChanged,
            ui->maxValidDistanceSpinBox, &QDoubleSpinBox::setValue);
    connect(ui->maxValidDistanceSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            m_stereoSLS, &ZStereoSLS::setMaxValidDistance);

    connect(m_stereoSLS, &ZStereoSLS::debugShowDecodedImagesChanged,
            ui->debugShowDecodedImagesCheckBox, &QCheckBox::setChecked);
    connect(ui->debugShowDecodedImagesCheckBox, &QCheckBox::toggled,
            m_stereoSLS, &ZStereoSLS::setDebugShowDecodedImages);

    connect(m_stereoSLS, &ZStereoSLS::debugShowFringesChanged,
            ui->debugShowFringesCheckBox, &QCheckBox::setChecked);
    connect(ui->debugShowFringesCheckBox, &QCheckBox::toggled,
            m_stereoSLS, &ZStereoSLS::setDebugShowFringes);

    ui->maxValidDistanceSpinBox->setValue(m_stereoSLS->maxValidDistance());
    ui->debugShowDecodedImagesCheckBox->setChecked(m_stereoSLS->debugShowDecodedImages());
    ui->debugShowFringesCheckBox->setChecked(m_stereoSLS->debugShowFringes());

    /// Left camera
    QMenu *leftCameraMenu = new QMenu(this);
    QAction *leftPreviewAction = leftCameraMenu->addAction(tr("Preview..."));
    connect(leftPreviewAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->leftCamera()->camera();
        auto *previewDialog = new Z3D::ZCameraPreviewer(camera);
        previewDialog->show();
    });
    QAction *leftSettingsAction = leftCameraMenu->addAction(tr("Settings..."));
    connect(leftSettingsAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->leftCamera()->camera();
        camera->showSettingsDialog();
    });
    QAction *leftCalibrationAction = leftCameraMenu->addAction(tr("Calibration..."));
    connect(leftCalibrationAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->leftCamera();
        auto *calibrator = new Z3D::ZCameraCalibratorWidget(camera);
        calibrator->show();
    });
    ui->leftCameraButton->setMenu(leftCameraMenu);

    /// Right camera
    QMenu *rightCameraMenu = new QMenu(this);
    QAction *rightPreviewAction = rightCameraMenu->addAction(tr("Preview..."));
    connect(rightPreviewAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->rightCamera()->camera();
        auto *previewDialog = new Z3D::ZCameraPreviewer(camera);
        previewDialog->show();
    });
    QAction *rightSettingsAction = rightCameraMenu->addAction(tr("Settings..."));
    connect(rightSettingsAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->rightCamera()->camera();
        camera->showSettingsDialog();
    });
    QAction *rightCalibrationAction = rightCameraMenu->addAction(tr("Calibration..."));
    connect(rightCalibrationAction, &QAction::triggered, [&](){
        auto camera = m_stereoSLS->rightCamera();
        auto *calibrator = new Z3D::ZCameraCalibratorWidget(camera);
        calibrator->show();
    });
    ui->rightCameraButton->setMenu(rightCameraMenu);
}

ZStereoSLSConfigWidget::~ZStereoSLSConfigWidget()
{
    delete ui;
}

} // namespace Z3D
