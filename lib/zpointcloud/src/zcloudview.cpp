#include "zcloudview.h"
#include "ui_zcloudview.h"

#include <Z3DCameraAcquisition>
#include <Z3DCameraCalibration>

#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"

#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/surface/mls.h"
//#include "pcl/surface/mls_omp.h"

#include "pcl/surface/gp3.h"
#include "pcl/surface/grid_projection.h"

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

#include "pcl/visualization/histogram_visualizer.h"

#if PCL_VERSION_COMPARE(>=,1,7,0)
#include "pcl/visualization/pcl_plotter.h"
#endif

#include "vtkCamera.h"
//#include "vtkLegendScaleActor.h"
#include "vtkRenderWindow.h"

#include <QDebug>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QTime>

#include <QCheckBox>
#include <QComboBox>
#include <QColorDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QSpinBox>
#include <QFormLayout>

/*
/// for Colormap "Jet" calculation
#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))
#define clamp(a, low, high) ((a > low) ? (a < high ? a : high) : low)
*/

template <typename T>
bool writeData(QString fileName, const std::vector<T> &errorsInliers) {

    //! FIXME
    return true;

    QFile file(fileName);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        for (int i=0; i<errorsInliers.size(); ++i)
            out << errorsInliers[i] << "\n";
        out.flush();
        file.close();
        return true;
    }
    return false;
}

namespace Z3D
{

ZCloudView::ZCloudView(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ZCloudView),
    m_pclViewer(new pcl::visualization::PCLVisualizer("3D", false))
{
    ui->setupUi(this);

    /// setup QVTKWidget
    vtkSmartPointer<vtkRenderWindow> renderWindow = m_pclViewer->getRenderWindow();
    ui->qvtkWidget->SetRenderWindow(renderWindow);

    /// these are useful to add to make the controls more like pcd_viewer
    m_pclViewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    m_pclViewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);

    /// default background color
    m_currentBackgroundColor.setRedF(0.1);
    m_currentBackgroundColor.setGreenF(0.1);
    m_currentBackgroundColor.setBlueF(0.1);
    m_pclViewer->setBackgroundColor(m_currentBackgroundColor.redF(),
                                    m_currentBackgroundColor.greenF(),
                                    m_currentBackgroundColor.blueF());

    /// init some default camera parameters
    m_pclViewer->initCameraParameters();

#if PCL_VERSION_COMPARE(>=,1,7,0)
    /// disable FPS text
    m_pclViewer->setShowFPS(false);
#endif

    /// set orthographic projection
    vtkSmartPointer<vtkRendererCollection> rendererCollection = m_pclViewer->getRenderWindow()->GetRenderers();
    rendererCollection->InitTraversal();
    vtkSmartPointer<vtkRenderer> renderer = rendererCollection->GetNextItem();
    vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
    camera->ParallelProjectionOn();
    camera->SetParallelScale(1000);

    /// add scale
    /*vtkSmartPointer<vtkLegendScaleActor> grid_actor_ = vtkSmartPointer<vtkLegendScaleActor>::New();
    grid_actor_->TopAxisVisibilityOn ();
    renderer->AddViewProp (grid_actor_);*/
}

ZCloudView::~ZCloudView()
{
    delete ui;
}

void ZCloudView::showCameraSnapshot(Z3D::ZCalibratedCamera::Ptr pCamera)
{
    Z3D::ZCameraCalibration::Ptr calibration = pCamera->calibration();

    cv::Mat intensityImg = pCamera->camera()->getSnapshot()->cvMat().clone();

    const int &width = calibration->sensorWidth();
    const int &height = calibration->sensorHeight();

    PointCloudPCLPtr cloud(new PointCloudPCL);

    int lod_step = 4; //ui->lodStepSpinBox->value();
    float imageScale = 50; //ui->imageScaleSpinBox->value();

    /// Fill in the cloud data
    cloud->width  = std::ceil(float(width)/lod_step) * std::ceil(float(height)/lod_step);
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    int i = 0;

    for (int x = 0; x < width; x += lod_step) {
        for (int y = 0; y < height; y += lod_step) {
            /// get camera ray for current pixel
            cv::Vec3d origin, ray;
            //calibration->getWorldRayForPixel(x, y, origin, ray);
            calibration->getRayForPixel(x, y, origin, ray);

            /// scale and rotate
            //cv::Vec3d point = ray * imageScale; // world ray is rotated, but it's a unitary vector, so the image plane doesn't look like a plane...
            cv::Vec3d point = calibration->rotation() * ray * imageScale;

            /// translate
            point += origin;

            cloud->points[i].x = point[0];
            cloud->points[i].y = point[1];
            cloud->points[i].z = point[2];

            /// color
            if (intensityImg.channels() == 1) {
                cloud->points[i].intensity = intensityImg.at<unsigned char>(y,x);
                /*unsigned int iB = intensityImg.at<unsigned char>(y,x);
                uint32_t rgb = (static_cast<uint32_t>(iB) << 16 |
                                static_cast<uint32_t>(iB) << 8 | static_cast<uint32_t>(iB));
                cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);*/
            } else {
                cv::Vec3b intensity = intensityImg.at<cv::Vec3b>(y, x);
                /*uchar blue = intensity.val[0];
                uchar green = intensity.val[1];
                uchar red = intensity.val[2];*/
                uint32_t rgb = (static_cast<uint32_t>(intensity.val[0]) << 16 |
                                static_cast<uint32_t>(intensity.val[1]) <<  8 |
                                static_cast<uint32_t>(intensity.val[2]));
                cloud->points[i].intensity /*rgb*/ = *reinterpret_cast<float*>(&rgb);
            }

            ++i;
        }
    }

    cloud->width  = i;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    m_pclViewer->removePointCloud( qPrintable(QString("snapshot_camera_%1").arg(pCamera->camera()->uuid())) );
    //pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(cloud);
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> rgb(cloud, "intensity");
    m_pclViewer->addPointCloud<PointType>(cloud, rgb, qPrintable(QString("snapshot_camera_%1").arg(pCamera->camera()->uuid())));

    drawCameraFrustrum(pCamera, imageScale);
}

void ZCloudView::drawCameraFrustrum(Z3D::ZCalibratedCamera::Ptr cam, float scale)
{
    Z3D::ZCameraCalibration::Ptr calibration = cam->calibration();

    cv::Vec3d point;

    const int nx = calibration->sensorWidth();
    const int ny = calibration->sensorHeight();

    for (int x = 0; x<nx; x += (nx-1)) {
        for (int y = 0; y<ny; y += (ny-1)) {
            // get camera ray for current pixel
            cv::Vec3d origin, ray;
            calibration->getWorldRayForPixel(x, y, origin, ray);

            // scale
            point = ray * scale;
            // rotate
            //point = calibration->rotation() * point; //world ray already rotated!
            // translate
            point += origin;

            pcl::PointXYZ p1, p2;
            p1.x = origin[0];
            p1.y = origin[1];
            p1.z = origin[2];

            p2.x = point[0];
            p2.y = point[1];
            p2.z = point[2];

            const std::string id = qPrintable(QString("camera_%3_line_for_x%1_y%2").arg(x).arg(y).arg(cam->camera()->uuid()));
            m_pclViewer->removeShape(id);
            m_pclViewer->addLine(p1, p2, 1,0,0, id);
        }
    }
}

void ZCloudView::addPointCloud(PointCloudPCLPtr cloud, const QString &id)
{
    m_pointCloud = Z3D::ZPointCloud::Ptr(new Z3D::ZPointCloud(cloud));
    m_pointCloud->setId(id);

    /// Display
    ui->actionViewPoints->setChecked(false);
    ui->actionViewPoints->setChecked(true);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> ZCloudView::getViewer()
{
    return m_pclViewer;
}

void ZCloudView::on_actionLoadPointCloud_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Load point cloud"),
                                                    "",
                                                    tr("Point Cloud Library file (*.pcd)"
                                                       ";;PLY file (*.ply)"));

    if (fileName.isEmpty() || fileName.isNull())
        return;

    PointCloudPCLPtr cloud(new PointCloudPCL);

    int returnStatus = -1;
    if (fileName.endsWith(".pcd", Qt::CaseInsensitive)) {
        returnStatus = pcl::io::loadPCDFile(qPrintable(fileName), *cloud);
    } else if (fileName.endsWith(".ply", Qt::CaseInsensitive)) {
        returnStatus = pcl::io::loadPLYFile(qPrintable(fileName), *cloud);
    } else if (fileName.endsWith(".asc", Qt::CaseInsensitive)) {
        //! TODO implementar esto, creo que pcl >= 1.7 tiene un ASCII reader
        qWarning() << "*.asc file loading is not implemented yet!";
    } else {
        qWarning() << "Unknown file extension:" << fileName;
    }

    if (returnStatus != 0)
        qWarning() << "Unable to load cloud from file:" << fileName;
    else
        addPointCloud(cloud, fileName);
}

void ZCloudView::on_actionSavePointCloudAs_triggered()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save point cloud as..."),
                                                    ".",
                                                    tr("Point Cloud Library file (*.pcd)"
                                                       ";;PLY file (*.ply)"));

    if (fileName.isEmpty() || fileName.isNull())
        return;

    int returnStatus = -1;
    if (fileName.endsWith(".pcd", Qt::CaseInsensitive)) {
        /// true == binary mode
        returnStatus = pcl::io::savePCDFile(qPrintable(fileName), *m_pointCloud->pclPointCloud(), true);
    } else if (fileName.endsWith(".ply", Qt::CaseInsensitive)) {
        returnStatus = pcl::io::savePLYFile(qPrintable(fileName), *m_pointCloud->pclPointCloud());
    } else if (fileName.endsWith(".asc", Qt::CaseInsensitive)) {
        //! TODO implementar esto, creo que pcl >= 1.7 tiene un ASCII reader
        qWarning() << "*.asc file saving is not implemented yet!";
    } else {
        qWarning() << "Unknown file extension:" << fileName;
    }

    if (returnStatus != 0)
        qWarning() << "Unable to save cloud to file:" << fileName;
}

void ZCloudView::on_actionCloseAllPointClouds_triggered()
{
    m_pclViewer->removeAllShapes();
    m_pclViewer->removeAllPointClouds(); //! TODO: averiguar si hacen falta los dos?

    m_pclViewer->getRenderWindow()->Render();
}

void ZCloudView::on_actionToolsStatisticalOutlierRemoval_triggered()
{
    QDialog dialog(this);
    dialog.setWindowTitle(tr("Statistical Outlier removal"));

    /// Use a layout allowing to have a label next to each field
    QFormLayout form(&dialog);

    /// Add the editors with their respective labels
    QSpinBox *meanKSpinBox = new QSpinBox(&dialog);
    meanKSpinBox->setValue(10);
    form.addRow(tr("Mean K"), meanKSpinBox);

    QDoubleSpinBox *stdDevThresholdSpinBox = new QDoubleSpinBox(&dialog);
    stdDevThresholdSpinBox->setValue(2.0);
    form.addRow(tr("STD dev. threshold"), stdDevThresholdSpinBox);

    /// Add some standard buttons (Cancel/Ok) at the bottom of the dialog
    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, &dialog);
    form.addRow(&buttonBox);
    QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
    QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));

    /// Show the dialog as modal
    if (dialog.exec() != QDialog::Accepted)
        return;

    QTime time;
    time.start();

    /// Create the filtering object
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(m_pointCloud->pclPointCloud());

    /**
     * The number of neighbors to analyze for each point is set to 50, and the
     * standard deviation multiplier to 1. All points who have a distance larger
     * than 1 standard deviation of the mean distance will be removed
     */
    sor.setMeanK(meanKSpinBox->value());
    sor.setStddevMulThresh(stdDevThresholdSpinBox->value());

    PointCloudPCLPtr cloudFiltered(new PointCloudPCL);
    sor.filter(*cloudFiltered);

    qDebug() << "SOR finished, point cloud reduced to" << cloudFiltered->size()
             << "from" << m_pointCloud->pclPointCloud()->size() << "in" << time.elapsed() << "msecs";

    m_pointCloud->setPclPointCloud(cloudFiltered);

    //m_pclViewer->updatePointCloud(m_pointCloud, "cloud");
    m_pclViewer->removePointCloud(qPrintable(m_pointCloud->id()));
    //pcl::visualization::PointCloudColorHandlerGenericField<PointType> rgb(m_pointCloud, "intensity");
    m_pclViewer->addPointCloud<PointType>(m_pointCloud->pclPointCloud(),
                                          *m_pointCloud->colorHandler(),
                                          qPrintable(m_pointCloud->id()));
}

void ZCloudView::on_actionToolsMovingLeastSquares_triggered()
{
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    //pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal> mls;



    QDialog paramsDialog(this);
    paramsDialog.setWindowTitle(tr("MLS - Moving Least Squares"));

    /// Use a layout allowing to have a label next to each field
    QFormLayout paramsDialogForm(&paramsDialog);

    /// Add the editors with their respective labels
    QDoubleSpinBox *searchRadiusSpinBox = new QDoubleSpinBox(&paramsDialog);
    searchRadiusSpinBox->setRange(0, 10000);
    searchRadiusSpinBox->setSingleStep(0.1);
    searchRadiusSpinBox->setValue(0.6); // 0.6 [mm], 600 [um]
    paramsDialogForm.addRow(tr("Search radius"), searchRadiusSpinBox);

    QCheckBox *polynomialFitCheckBox = new QCheckBox(&paramsDialog);
    polynomialFitCheckBox->setChecked(false);
    paramsDialogForm.addRow(tr("Polynomial fit"), polynomialFitCheckBox);

    QSpinBox *polynomialOrderSpinBox = new QSpinBox(&paramsDialog);
    polynomialOrderSpinBox->setRange(1, 10);
    polynomialOrderSpinBox->setValue(2);
    paramsDialogForm.addRow(tr("Polynomial order"), polynomialOrderSpinBox);

    QCheckBox *computeNormalsCheckBox = new QCheckBox(&paramsDialog);
    computeNormalsCheckBox->setChecked(true);
    paramsDialogForm.addRow(tr("Compute normals"), computeNormalsCheckBox);

    QComboBox *upsamplingMethodComboBox = new QComboBox(&paramsDialog);
    upsamplingMethodComboBox->addItem("None", mls.NONE);
    upsamplingMethodComboBox->addItem("Random uniform density", mls.RANDOM_UNIFORM_DENSITY);
    upsamplingMethodComboBox->addItem("Voxel grid dilation", mls.VOXEL_GRID_DILATION);
    paramsDialogForm.addRow(tr("Upsampling method"), upsamplingMethodComboBox);

    QSpinBox *pointDensitySpinBox = new QSpinBox(&paramsDialog);
    pointDensitySpinBox->setRange(1, 100);
    pointDensitySpinBox->setValue(4);
    paramsDialogForm.addRow(tr("Point density (Random uniform density)"), pointDensitySpinBox);

    QSpinBox *dilationIterationsSpinBox = new QSpinBox(&paramsDialog);
    dilationIterationsSpinBox->setRange(1, 100);
    dilationIterationsSpinBox->setValue(4);
    paramsDialogForm.addRow(tr("Dilation iterations (Voxel Grid)"), dilationIterationsSpinBox);

    QDoubleSpinBox *voxelSizeSpinBox = new QDoubleSpinBox(&paramsDialog);
    voxelSizeSpinBox->setRange(0, 100);
    voxelSizeSpinBox->setSingleStep(0.1);
    voxelSizeSpinBox->setValue(0.3); // 0.6 [mm], 600 [um]
    paramsDialogForm.addRow(tr("Voxel size (Voxel Grid)"), voxelSizeSpinBox);

    /// Add some standard buttons (Cancel/Ok) at the bottom of the dialog
    QDialogButtonBox paramsButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, &paramsDialog);
    paramsDialogForm.addRow(&paramsButtonBox);
    QObject::connect(&paramsButtonBox, SIGNAL(accepted()), &paramsDialog, SLOT(accept()));
    QObject::connect(&paramsButtonBox, SIGNAL(rejected()), &paramsDialog, SLOT(reject()));

    /// Show the dialog as modal
    if (paramsDialog.exec() != QDialog::Accepted)
        return;

    QTime time;
    time.start();

    /// Init object (second point type is for the normals, even if unused)
    /// Had to create other cloud because it seems it doesn't work for PointXYZI
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*m_pointCloud->pclPointCloud(), *cloud2);
    pcl::PointCloud<pcl::PointNormal>::Ptr normals2(new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    qDebug() << "MLS preparation finished in" << time.elapsed() << "msecs";
    time.restart();

    /// Set parameters
    mls.setInputCloud(cloud2);
    mls.setSearchMethod(tree);

    /// Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting.
    /// Calling this method resets the squared Gaussian parameter to radius * radius !
    mls.setSearchRadius(searchRadiusSpinBox->value());

    /// Set the parameter used for distance based weighting of neighbors
    /// (the square of the search radius works best in general).
    //mls.setSqrGaussParam();

    /// Set whether the algorithm should also store the normals computed
    /// This is optional, but need a proper output cloud type
    mls.setComputeNormals(computeNormalsCheckBox->isChecked());

    /// Sets whether the surface and normal are approximated using a polynomial, or only via tangent estimation.
    mls.setPolynomialFit(polynomialFitCheckBox->isChecked());

    if (mls.getPolynomialFit()) {
        /// Set the order of the polynomial to be fit.
        mls.setPolynomialOrder(polynomialOrderSpinBox->value());
    }

    bool ok;
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod method =
            (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod) upsamplingMethodComboBox->itemData(upsamplingMethodComboBox->currentIndex()).toInt(&ok);
    if (ok) {
        mls.setUpsamplingMethod(method);

        switch (method) {
        case mls.NONE:
            break;
        case mls.VOXEL_GRID_DILATION:
            /// Set the number of dilation steps of the voxel grid
            /// Used only in the VOXEL_GRID_DILATION upsampling method
            mls.setDilationIterations(dilationIterationsSpinBox->value());

            /// Set the voxel size for the voxel grid
            /// Used only in the VOXEL_GRID_DILATION upsampling method
            mls.setDilationVoxelSize(voxelSizeSpinBox->value());

            break;
        case mls.RANDOM_UNIFORM_DENSITY:
            /// Set the parameter that specifies the desired number of points within the search radius
            /// Used only in the case of RANDOM_UNIFORM_DENSITY upsampling
            mls.setPointDensity(pointDensitySpinBox->value());

            break;
        case mls.SAMPLE_LOCAL_PLANE:
            /// Set the step size for the local plane sampling
            /// Used only in the case of SAMPLE_LOCAL_PLANE upsampling
            //mls.setUpsamplingStepSize();

            /// Set the radius of the circle in the local point plane that will be sampled
            /// Used only in the case of SAMPLE_LOCAL_PLANE upsampling
            //mls.setUpsamplingRadius();

            break;
        default:
            qWarning() << "unknown upsampling method:" << method;
            return;
        }
    } else {
        qWarning() << "error! cannot obtaion/parse upsampling method" << upsamplingMethodComboBox->itemData(upsamplingMethodComboBox->currentIndex());
    }


    /// Reconstruct
    /// Output has the SurfeType (Point+Normal) type in order to store the normals calculated by MLS
    mls.process(*normals2);

    qDebug() << "MLS processing finished in" << time.elapsed() << "msecs";
    time.restart();

    /// Update original cloud with smoothed cloud and new normals
    PointCloudPCLPtr newCloud(new PointCloudPCL);
    SurfaceNormalsPtr newNormals(new SurfaceNormals);
    pcl::copyPointCloud(*normals2, *newCloud);
    m_pointCloud->setPclPointCloud(newCloud);
    pcl::copyPointCloud(*normals2, *newNormals);
    m_pointCloud->setPclNormals(newNormals);

    qDebug() << "MLS results copyied in" << time.elapsed() << "msecs";

    /// Update display
    if (ui->actionViewPoints->isChecked()) {
        ui->actionViewPoints->setChecked(false);
        ui->actionViewPoints->setChecked(true);
    }

    if (ui->actionViewNormals->isChecked()) {
        ui->actionViewNormals->setChecked(false);
        ui->actionViewNormals->setChecked(true);
    }
}

void ZCloudView::on_actionToolsGreedyTriangulation_triggered()
{
    QDialog paramsDialog(this);
    paramsDialog.setWindowTitle(tr("Greedy triangulation"));

    /// Use a layout allowing to have a label next to each field
    QFormLayout paramsDialogForm(&paramsDialog);

    /// Add the editors with their respective labels
    QDoubleSpinBox *maxEdgeLenghtSpinBox = new QDoubleSpinBox(&paramsDialog);
    maxEdgeLenghtSpinBox->setRange(0, 10000);
    maxEdgeLenghtSpinBox->setSingleStep(0.1);
    maxEdgeLenghtSpinBox->setValue(0.6); // 0.6 [mm], 600 [um]
    paramsDialogForm.addRow(tr("Max. edge lenght"), maxEdgeLenghtSpinBox);

    QDoubleSpinBox *muSpinBox = new QDoubleSpinBox(&paramsDialog);
    muSpinBox->setRange(0, 10000);
    muSpinBox->setSingleStep(0.1);
    muSpinBox->setValue(3);
    paramsDialogForm.addRow(tr("Mu coeff."), muSpinBox);

    QSpinBox *maxNearestNeighborsSpinBox = new QSpinBox(&paramsDialog);
    maxNearestNeighborsSpinBox->setRange(1, 10000);
    maxNearestNeighborsSpinBox->setValue(30);
    paramsDialogForm.addRow(tr("Max. nearest neighbors"), maxNearestNeighborsSpinBox);

    QSpinBox *minAngleSpinBox = new QSpinBox(&paramsDialog);
    minAngleSpinBox->setRange(0, 360);
    minAngleSpinBox->setValue(5);
    paramsDialogForm.addRow(tr("Min. angle (triangle)"), minAngleSpinBox);

    QSpinBox *maxAngleSpinBox = new QSpinBox(&paramsDialog);
    maxAngleSpinBox->setRange(1, 360);
    maxAngleSpinBox->setValue(170);
    paramsDialogForm.addRow(tr("Max. angle (triangle)"), maxAngleSpinBox);

    QSpinBox *maxSurfaceAngleSpinBox = new QSpinBox(&paramsDialog);
    maxSurfaceAngleSpinBox->setRange(1, 360);
    maxSurfaceAngleSpinBox->setValue(90);
    paramsDialogForm.addRow(tr("Max. surface angle"), maxSurfaceAngleSpinBox);

    QCheckBox *normalConsistencyCheckBox = new QCheckBox(&paramsDialog);
    normalConsistencyCheckBox->setChecked(false);
    paramsDialogForm.addRow(tr("Normal consistency"), normalConsistencyCheckBox);

    /// Add some standard buttons (Cancel/Ok) at the bottom of the dialog
    QDialogButtonBox paramsButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, &paramsDialog);
    paramsDialogForm.addRow(&paramsButtonBox);
    QObject::connect(&paramsButtonBox, SIGNAL(accepted()), &paramsDialog, SLOT(accept()));
    QObject::connect(&paramsButtonBox, SIGNAL(rejected()), &paramsDialog, SLOT(reject()));

    /// Show the dialog as modal
    if (paramsDialog.exec() != QDialog::Accepted)
        return;



    QTime time;
    time.start();

    /// Create search tree*
    pcl::search::KdTree<SurfelType>::Ptr tree2(new pcl::search::KdTree<SurfelType>);
    tree2->setInputCloud (m_pointCloud->pclSurfaceElements());

    /// Initialize object
    pcl::GreedyProjectionTriangulation<SurfelType> gp3;

    /// Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(maxEdgeLenghtSpinBox->value());

    /// Set typical values for the parameters
    gp3.setMaximumNearestNeighbors(maxNearestNeighborsSpinBox->value());
    gp3.setMu(muSpinBox->value());

    /// min and max triangle angles
    gp3.setMinimumAngle(minAngleSpinBox->value() * M_PI/180.);
    gp3.setMaximumAngle(maxAngleSpinBox->value() * M_PI/180.);

    /// dont connect surfaces if the angle is bigger than 60 deg
    gp3.setMaximumSurfaceAngle(maxSurfaceAngleSpinBox->value() * M_PI/180);

    gp3.setNormalConsistency(normalConsistencyCheckBox->isChecked());

    /// Get result
    gp3.setInputCloud(m_pointCloud->pclSurfaceElements());
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(*m_pointCloud->pclSurfaceMesh());
    qDebug() << "finished greedy triangulation in" << time.elapsed() << "msecs";

    /// Display
    ui->actionViewSurface->setChecked(false);
    ui->actionViewSurface->setChecked(true);
}

void ZCloudView::on_actionToolsGridProjection_triggered()
{
    /// Create search tree*
    pcl::search::KdTree<SurfelType>::Ptr tree2(new pcl::search::KdTree<SurfelType>);
    tree2->setInputCloud (m_pointCloud->pclSurfaceElements());

    /// Initialize objects
    pcl::GridProjection<SurfelType> gbpolygon;

    /// Set parameters
    gbpolygon.setResolution(0.05); //0.005
    gbpolygon.setPaddingSize(3);
    gbpolygon.setNearestNeighborNum(100);
    gbpolygon.setMaxBinarySearchLevel(10);

    /// Get result
    gbpolygon.setInputCloud(m_pointCloud->pclSurfaceElements());
    gbpolygon.setSearchMethod(tree2);
    gbpolygon.reconstruct(*m_pointCloud->pclSurfaceMesh());

    /// Display
    ui->actionViewSurface->setChecked(false);
    ui->actionViewSurface->setChecked(true);
}

void ZCloudView::on_actionDownsample_triggered()
{
    QDialog paramsDialog(this);
    paramsDialog.setWindowTitle(tr("Downsample cloud"));

    /// Use a layout allowing to have a label next to each field
    QFormLayout paramsDialogForm(&paramsDialog);

    /// Add the editors with their respective labels
    QDoubleSpinBox *leafSizeSpinBox = new QDoubleSpinBox(&paramsDialog);
    leafSizeSpinBox->setRange(0, 10000);
    leafSizeSpinBox->setSingleStep(0.1);
    leafSizeSpinBox->setValue(0.1); // 0.1 == 100um (if we work in mm)
    paramsDialogForm.addRow(tr("Leaf size"), leafSizeSpinBox);

    /// Add some standard buttons (Cancel/Ok) at the bottom of the dialog
    QDialogButtonBox paramsButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, &paramsDialog);
    paramsDialogForm.addRow(&paramsButtonBox);
    QObject::connect(&paramsButtonBox, SIGNAL(accepted()), &paramsDialog, SLOT(accept()));
    QObject::connect(&paramsButtonBox, SIGNAL(rejected()), &paramsDialog, SLOT(reject()));

    /// Show the dialog as modal
    if (paramsDialog.exec() != QDialog::Accepted)
        return;


    double leafSize = leafSizeSpinBox->value();

    PointCloudPCLPtr cloudFiltered(new PointCloudPCL);

    QTime time;
    time.start();

    /// Create the filtering object
    pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setInputCloud (m_pointCloud->pclPointCloud());
    voxelgrid.setLeafSize (leafSize, leafSize, leafSize);

    voxelgrid.filter(*cloudFiltered);

    qDebug() << "reduced cloud to" << cloudFiltered->size()
             << "points (was" << m_pointCloud->pclPointCloud()->size()
             << "points) in" << time.elapsed() << "msecs";

    m_pointCloud->setPclPointCloud(cloudFiltered);//pcl::copyPointCloud(*cloudFiltered, *m_pointCloud);

    /// Update display
    if (ui->actionViewPoints->isChecked()) {
        ui->actionViewPoints->setChecked(false);
        ui->actionViewPoints->setChecked(true);
    }
}

void ZCloudView::on_actionViewCoordinateSystem_toggled(bool arg1)
{
    if (arg1) {
        m_pclViewer->addCoordinateSystem(25);

#if PCL_VERSION_COMPARE(>=,1,7,0)
        /// Adds a widget which shows an interactive axes display for orientation
        m_pclViewer->addOrientationMarkerWidgetAxes(m_pclViewer->getRenderWindow()->GetInteractor());
#endif
    } else {
        m_pclViewer->removeCoordinateSystem();

#if PCL_VERSION_COMPARE(>=,1,7,0)
        /// Removes the widget which shows an interactive axes display for orientation
        m_pclViewer->removeOrientationMarkerWidgetAxes();
#endif
    }

    ui->qvtkWidget->update();
}

void ZCloudView::on_actionToolsFitCylinder_triggered()
{
    QDialog paramsDialog(this);
    paramsDialog.setWindowTitle(tr("Fit cylinder"));

    /// Use a layout allowing to have a label next to each field
    QFormLayout paramsDialogForm(&paramsDialog);

    /// Add the editors with their respective labels
    QSpinBox *maxIterationsSpinBox = new QSpinBox(&paramsDialog);
    maxIterationsSpinBox->setRange(1, 10000);
    maxIterationsSpinBox->setValue(100);
    paramsDialogForm.addRow(tr("Max. iterations"), maxIterationsSpinBox);

    QDoubleSpinBox *normalDistanceWeightSpinBox = new QDoubleSpinBox(&paramsDialog);
    normalDistanceWeightSpinBox->setRange(0, 1);
    normalDistanceWeightSpinBox->setSingleStep(0.1);
    normalDistanceWeightSpinBox->setValue(0.1);
    paramsDialogForm.addRow(tr("Normal distance weight"), normalDistanceWeightSpinBox);

    QDoubleSpinBox *distanceThresholdSpinBox = new QDoubleSpinBox(&paramsDialog);
    distanceThresholdSpinBox->setRange(0, 10000);
    distanceThresholdSpinBox->setSingleStep(0.1);
    distanceThresholdSpinBox->setValue(0.5);
    paramsDialogForm.addRow(tr("Distance threshold"), distanceThresholdSpinBox);

    QDoubleSpinBox *minRadiusSpinBox = new QDoubleSpinBox(&paramsDialog);
    minRadiusSpinBox->setRange(0, 10000);
    minRadiusSpinBox->setValue(0);
    paramsDialogForm.addRow(tr("Min. radius"), minRadiusSpinBox);

    QDoubleSpinBox *maxRadiusSpinBox = new QDoubleSpinBox(&paramsDialog);
    maxRadiusSpinBox->setRange(0, 10000);
    maxRadiusSpinBox->setValue(100);
    paramsDialogForm.addRow(tr("Max. radius"), maxRadiusSpinBox);

    QCheckBox *optimizeCoeffsCheckBox = new QCheckBox(&paramsDialog);
    optimizeCoeffsCheckBox->setChecked(true);
    paramsDialogForm.addRow(tr("Optimize coefficients"), optimizeCoeffsCheckBox);

    /// Add some standard buttons (Cancel/Ok) at the bottom of the dialog
    QDialogButtonBox paramsButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, &paramsDialog);
    paramsDialogForm.addRow(&paramsButtonBox);
    QObject::connect(&paramsButtonBox, SIGNAL(accepted()), &paramsDialog, SLOT(accept()));
    QObject::connect(&paramsButtonBox, SIGNAL(rejected()), &paramsDialog, SLOT(reject()));

    /// Show the dialog as modal
    if (paramsDialog.exec() != QDialog::Accepted)
        return;


    //pcl::NormalEstimation<PointType, NormalType> ne;
    pcl::SACSegmentationFromNormals<PointType, NormalType> seg;
    //pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());

    /// Datasets
    //pcl::PointCloud<NormalType>::Ptr cloud_normals (new pcl::PointCloud<NormalType>);
    pcl::ModelCoefficients::Ptr cylinderCoefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr cylinderInlierIndices (new pcl::PointIndices);

    QTime time;
    time.start();

    /// Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (optimizeCoeffsCheckBox->isChecked());
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (normalDistanceWeightSpinBox->value());
    seg.setMaxIterations (maxIterationsSpinBox->value());
    seg.setDistanceThreshold (distanceThresholdSpinBox->value());
    seg.setRadiusLimits (minRadiusSpinBox->value(), maxRadiusSpinBox->value());
    seg.setInputCloud (m_pointCloud->pclPointCloud());
    seg.setInputNormals (m_pointCloud->pclNormals());

    /// Obtain the cylinder inliers and coefficients
    seg.segment (*cylinderInlierIndices, *cylinderCoefficients);
    qDebug() << "cylinder segmentation:" << time.elapsed() << "msecs";

    /// Display results
    std::cerr << "Cylinder coefficients: " << *cylinderCoefficients << std::endl;




    if (!cylinderCoefficients->values.size()) {
        qWarning() << "cylinder could not be found";
        //QMessageBox::information()
        return;
    }

    /// cylinder radius
    float cylinderRadius = cylinderCoefficients->values[6];

    /// compute error stats
    ///
    /// cylinder center point
    /*cv::Vec3f cylinderAxisPoint(
            cylinderCoefficients->values[0],
            cylinderCoefficients->values[1],
            cylinderCoefficients->values[2]);*/

    /// cylinder direction
    cv::Vec3f cylinderAxisVector(
            cylinderCoefficients->values[3],
            cylinderCoefficients->values[4],
            cylinderCoefficients->values[5]);

    /// to know where (and how long) to display the cylinder later
    double minAxialDispl = DBL_MAX;
    double maxAxialDispl = DBL_MIN;

    std::vector<float> errorsInliers;
    errorsInliers.reserve(cylinderInlierIndices->indices.size());
    for (std::vector<int>::const_iterator it = cylinderInlierIndices->indices.begin();
            it != cylinderInlierIndices->indices.end();
            ++it) {
        /// cloud point
        cv::Vec3f pointVector(
                m_pointCloud->pclPointCloud()->points[*it].x - cylinderCoefficients->values[0],
                m_pointCloud->pclPointCloud()->points[*it].y - cylinderCoefficients->values[1],
                m_pointCloud->pclPointCloud()->points[*it].z - cylinderCoefficients->values[2]);

        float proj = pointVector.dot(cylinderAxisVector);

        /// to know where (and how long) to display the cylinder later
        if (minAxialDispl > proj)
            minAxialDispl = proj;

        if (maxAxialDispl < proj)
            maxAxialDispl = proj;

        float pointRadious = cv::norm(pointVector - (proj * cylinderAxisVector));

        errorsInliers.push_back( pointRadious - cylinderRadius );
    }

    double mean, stddev;
    //pcl::getMeanStd (const std::vector< float > &values, double &mean, double &stddev)
    pcl::getMeanStd(errorsInliers, mean, stddev);


    if (!writeData(QString("%1_%2_cylinderFitErrors.txt")
              .arg(m_pointCloud->id())
              .arg(QDateTime::currentDateTime().toString("yyyyMMddHHmmss")),
              errorsInliers))
        qWarning() << "unable to save error data";




    boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer> histogram (new pcl::visualization::PCLHistogramVisualizer);
    //sensor_msgs::PointCloud2 rosCloud;
    //pcl::toROSMsg<PointType>(m_pointCloud->pclPointCloud(), rosCloud);
    histogram->addFeatureHistogram(*m_pointCloud->pclPointCloud(), "x", 2, "cloud_hist_x", 800, 600);
    histogram->setBackgroundColor(255,255,255);



#if PCL_VERSION_COMPARE(>=,1,7,0)
    pcl::visualization::PCLPlotter plotter;

    std::vector<double> errorsInliersD;
    errorsInliersD.reserve(errorsInliers.size());
    for (int i=0; i<errorsInliers.size(); ++i)
        errorsInliersD.push_back(errorsInliers[i]);

    plotter.addHistogramData(errorsInliersD, 100);
    plotter.setTitle(qPrintable(tr("Cylinder fit error distribution")));
    plotter.setXTitle(qPrintable(tr("Error")));
    plotter.setYTitle(qPrintable(tr("Points")));
    plotter.setShowLegend(false);
    plotter.spin();
#endif

    /// results dialog
    QDialog resultsDialog(this);
    resultsDialog.setWindowTitle(tr("Cylinder found"));

    /// Use a layout allowing to have a label next to each field
    QFormLayout resultsDialogForm(&resultsDialog);

    /// Add the editors with their respective labels
    QLineEdit *centerLabel = new QLineEdit(&resultsDialog);
    centerLabel->setReadOnly(true);
    centerLabel->setText(QString("[%1, %2, %3]")
            .arg(cylinderCoefficients->values[0])
            .arg(cylinderCoefficients->values[1])
            .arg(cylinderCoefficients->values[2]));
    resultsDialogForm.addRow(tr("Axis point"), centerLabel);

    QLineEdit *axisLabel = new QLineEdit(&resultsDialog);
    axisLabel->setReadOnly(true);
    axisLabel->setText(QString("[%1, %2, %3]")
            .arg(cylinderCoefficients->values[3])
            .arg(cylinderCoefficients->values[4])
            .arg(cylinderCoefficients->values[5]));
    resultsDialogForm.addRow(tr("Axis direction"), axisLabel);

    QLineEdit *radiusLabel = new QLineEdit(&resultsDialog);
    radiusLabel->setReadOnly(true);
    radiusLabel->setText(QString("%1").arg(cylinderRadius));
    resultsDialogForm.addRow(tr("Radius"), radiusLabel);

    QLineEdit *meanErrorLabel = new QLineEdit(&resultsDialog);
    meanErrorLabel->setReadOnly(true);
    meanErrorLabel->setText(QString("%1").arg(mean));
    resultsDialogForm.addRow(tr("Mean error"), meanErrorLabel);

    QLineEdit *stddevErrorLabel = new QLineEdit(&resultsDialog);
    stddevErrorLabel->setReadOnly(true);
    stddevErrorLabel->setText(QString("%1").arg(stddev));
    resultsDialogForm.addRow(tr("STD dev"), stddevErrorLabel);

    QCheckBox *showCylinderCheckBox = new QCheckBox(&resultsDialog);
    showCylinderCheckBox->setChecked(false);
    resultsDialogForm.addRow(tr("Display fitted cylinder"), showCylinderCheckBox);

    QCheckBox *showCylinderFitErrorCheckBox = new QCheckBox(&resultsDialog);
    showCylinderFitErrorCheckBox->setChecked(true);
    resultsDialogForm.addRow(tr("Colorize error"), showCylinderFitErrorCheckBox);

    QDoubleSpinBox *minErrorSpinBox = new QDoubleSpinBox(&resultsDialog);
    minErrorSpinBox->setRange(-10000, 10000);
    minErrorSpinBox->setSingleStep(0.1);
    minErrorSpinBox->setValue(-2.*stddev);
    resultsDialogForm.addRow(tr("Min. error"), minErrorSpinBox);

    QDoubleSpinBox *maxErrorSpinBox = new QDoubleSpinBox(&resultsDialog);
    maxErrorSpinBox->setRange(-10000, 10000);
    maxErrorSpinBox->setSingleStep(0.1);
    maxErrorSpinBox->setValue(2.*stddev);
    resultsDialogForm.addRow(tr("Max. error"), maxErrorSpinBox);

    /// Add some standard buttons (Cancel/Ok) at the bottom of the dialog
    QDialogButtonBox resultsButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, &resultsDialog);
    resultsDialogForm.addRow(&resultsButtonBox);
    QObject::connect(&resultsButtonBox, SIGNAL(accepted()), &resultsDialog, SLOT(accept()));
    QObject::connect(&resultsButtonBox, SIGNAL(rejected()), &resultsDialog, SLOT(reject()));

    /// Show the dialog as modal
    if (resultsDialog.exec() != QDialog::Accepted)
        return;


    if (showCylinderFitErrorCheckBox->isChecked()) {
        time.restart();

        float minError = minErrorSpinBox->value();
        float maxError = maxErrorSpinBox->value();
        //float rangeError = maxError - minError;
        //uint32_t pointColor;

        for (size_t i=0; i<m_pointCloud->pclPointCloud()->points.size(); ++i)
            m_pointCloud->pclPointCloud()->points[i].intensity = maxError;

        int i = 0;
        for (std::vector<int>::const_iterator it = cylinderInlierIndices->indices.begin();
                it != cylinderInlierIndices->indices.end();
                ++it, ++i) {


            float error = qMax(minError, qMin(maxError, errorsInliers[i]));

            /*qDebug() << "projection:" << proj
                     << "pointRadious" << pointRadious
                     << "cylinderRadious" << cylinderRadious
                     << "error" << error;*/

            m_pointCloud->pclPointCloud()->points[*it].intensity = error;

            /*
            float value = (error - minError) / rangeError;

            ////////////////////////////////////////////////////////////////////
            /// Returns the appropriate color from the "jet" colormap
            ///
            /// Implementing a Continuous "Jet" Colormap Function in GLSL
            /// http://www.metastine.com/?p=7
            ////////////////////////////////////////////////////////////////////
            float valuex4 = clamp(value, 0.f, 1.f) * 4.f;
            float r = min(valuex4 - 1.5f, -valuex4 + 4.5f);
            float g = min(valuex4 - 0.5f, -valuex4 + 3.5f);
            float b = min(valuex4 + 0.5f, -valuex4 + 2.5f);

            unsigned char ur = (unsigned char) 255.f * clamp(r, 0.f, 1.f); //R
            unsigned char ug = (unsigned char) 255.f * clamp(g, 0.f, 1.f); //G
            unsigned char ub = (unsigned char) 255.f * clamp(b, 0.f, 1.f); //B

            //qDebug() << error << value << r << g << b << ur << ug << ub;

            pointColor = (  static_cast<uint32_t>(ur) << 16 |
                            static_cast<uint32_t>(ug) <<  8 |
                            static_cast<uint32_t>(ub)       );

            m_pointCloud->points[*it].rgb = *reinterpret_cast<float*>(&pointColor);*/
        }

        qDebug() << "cylinder error colormap:" << time.elapsed() << "msecs";

        /// display point cloud
        //m_pclViewer->updatePointCloud(m_pointCloud, "cloud");
        m_pclViewer->removePointCloud(qPrintable(m_pointCloud->id()));
        //pcl::visualization::PointCloudColorHandlerGenericField<PointType> rgb(m_pointCloud, "intensity");
        m_pclViewer->addPointCloud<PointType>(m_pointCloud->pclPointCloud(),
                                              *m_pointCloud->colorHandler(),
                                              qPrintable(m_pointCloud->id()));
    }


    if (showCylinderCheckBox->isChecked()) {
        /// Move cylinder point to the extreme of the cloud
        for (int i=0; i<3; ++i)
            cylinderCoefficients->values[i] += cylinderCoefficients->values[i+3] * minAxialDispl;

        /// Scale direction vector to display the correct lenght
        float cylinderLength = maxAxialDispl - minAxialDispl;
        for (int i=3; i<6; ++i)
            cylinderCoefficients->values[i] *= cylinderLength;

        QString shapeId = QString("%1__fittedCylinder").arg(m_pointCloud->id());
        m_pclViewer->removeShape(qPrintable(shapeId));
        m_pclViewer->addCylinder(*cylinderCoefficients, qPrintable(shapeId));
    }
}

void ZCloudView::on_actionToolsFitPlane_triggered()
{
    QDialog paramsDialog(this);
    paramsDialog.setWindowTitle(tr("Fit plane"));

    /// Use a layout allowing to have a label next to each field
    QFormLayout paramsDialogForm(&paramsDialog);

    /// Add the editors with their respective labels
    QSpinBox *maxIterationsSpinBox = new QSpinBox(&paramsDialog);
    maxIterationsSpinBox->setRange(1, 10000);
    maxIterationsSpinBox->setValue(100);
    paramsDialogForm.addRow(tr("Max. iterations"), maxIterationsSpinBox);

    QDoubleSpinBox *normalDistanceWeightSpinBox = new QDoubleSpinBox(&paramsDialog);
    normalDistanceWeightSpinBox->setRange(0, 1);
    normalDistanceWeightSpinBox->setValue(0.1);
    paramsDialogForm.addRow(tr("Normal distance weight"), normalDistanceWeightSpinBox);

    QDoubleSpinBox *distanceThresholdSpinBox = new QDoubleSpinBox(&paramsDialog);
    distanceThresholdSpinBox->setRange(0, 10000);
    distanceThresholdSpinBox->setValue(1);
    paramsDialogForm.addRow(tr("Distance threshold"), distanceThresholdSpinBox);

    QCheckBox *optimizeCoeffsCheckBox = new QCheckBox(&paramsDialog);
    optimizeCoeffsCheckBox->setChecked(true);
    paramsDialogForm.addRow(tr("Optimize coefficients"), optimizeCoeffsCheckBox);

    /// Add some standard buttons (Cancel/Ok) at the bottom of the dialog
    QDialogButtonBox paramsButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, &paramsDialog);
    paramsDialogForm.addRow(&paramsButtonBox);
    QObject::connect(&paramsButtonBox, SIGNAL(accepted()), &paramsDialog, SLOT(accept()));
    QObject::connect(&paramsButtonBox, SIGNAL(rejected()), &paramsDialog, SLOT(reject()));

    /// Show the dialog as modal
    if (paramsDialog.exec() != QDialog::Accepted)
        return;




    pcl::SACSegmentationFromNormals<PointType, NormalType> seg;
    //pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());

    /// Datasets
    //pcl::PointCloud<NormalType>::Ptr cloud_normals (new pcl::PointCloud<NormalType>);
    pcl::ModelCoefficients::Ptr planeCoefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr planeInlierIndices (new pcl::PointIndices);

    QTime time;
    time.start();

    /// Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (optimizeCoeffsCheckBox->isChecked());
    seg.setModelType (pcl::SACMODEL_PLANE);
    //seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (normalDistanceWeightSpinBox->value());
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterationsSpinBox->value());
    seg.setDistanceThreshold (distanceThresholdSpinBox->value());
    seg.setInputCloud (m_pointCloud->pclPointCloud());
    seg.setInputNormals (m_pointCloud->pclNormals());

    /// Obtain the plane inliers and coefficients
    seg.segment (*planeInlierIndices, *planeCoefficients);
    qDebug() << "plane segmentation:" << time.elapsed() << "msecs";

    std::cerr << "Plane coefficients: " << *planeCoefficients << std::endl;



    if (!planeCoefficients->values.size()) {
        //QMessageBox::information
        return;
    }

    /// compute error stats
    ///
    /// plane distance from origin
    float planeDistance = - planeCoefficients->values[3];

    /// plane normal
    cv::Vec3f planeNormalVector(
            planeCoefficients->values[0],
            planeCoefficients->values[1],
            planeCoefficients->values[2]);

    std::vector<float> errorsInliers;
    errorsInliers.reserve(planeInlierIndices->indices.size());
    for (std::vector<int>::const_iterator it = planeInlierIndices->indices.begin();
            it != planeInlierIndices->indices.end();
            ++it) {
        /// cloud point
        cv::Vec3f pointVector(
                m_pointCloud->pclPointCloud()->points[*it].x,
                m_pointCloud->pclPointCloud()->points[*it].y,
                m_pointCloud->pclPointCloud()->points[*it].z);

        float proj = pointVector.dot(planeNormalVector);

        errorsInliers.push_back( proj - planeDistance );
    }

    double mean, stddev;
    //pcl::getMeanStd (const std::vector< float > &values, double &mean, double &stddev)
    pcl::getMeanStd(errorsInliers, mean, stddev);


    if (!writeData(QString("%1_%2_planeFitErrors.txt")
              .arg(m_pointCloud->id())
              .arg(QDateTime::currentDateTime().toString("yyyyMMddHHmmss")),
              errorsInliers))
        qWarning() << "unable to save error data";


#if PCL_VERSION_COMPARE(>=,1,7,0)
    pcl::visualization::PCLPlotter plotter;

    std::vector<double> errorsInliersD;
    errorsInliersD.reserve(errorsInliers.size());
    for (int i=0; i<errorsInliers.size(); ++i)
        errorsInliersD.push_back(errorsInliers[i]);

    plotter.addHistogramData(errorsInliersD, 100);
    plotter.setTitle(qPrintable(tr("Plane fit error distribution")));
    plotter.setXTitle(qPrintable(tr("Error")));
    plotter.setYTitle(qPrintable(tr("Points")));
    plotter.setShowLegend(false);
    plotter.spin();
#endif


    /// results dialog
    QDialog resultsDialog(this);
    resultsDialog.setWindowTitle(tr("Plane found"));

    /// Use a layout allowing to have a label next to each field
    QFormLayout resultsDialogForm(&resultsDialog);

    /// Add the editors with their respective labels
    QLineEdit *axisLabel = new QLineEdit(&resultsDialog);
    axisLabel->setReadOnly(true);
    axisLabel->setText(QString("[%1, %2, %3]")
            .arg(planeCoefficients->values[0])
            .arg(planeCoefficients->values[1])
            .arg(planeCoefficients->values[2]));
    resultsDialogForm.addRow(tr("Axis direction"), axisLabel);

    QLineEdit *wLabel = new QLineEdit(&resultsDialog);
    wLabel->setReadOnly(true);
    wLabel->setText(QString("%1")
            .arg(planeCoefficients->values[3]));
    resultsDialogForm.addRow(tr("Distance to origin"), wLabel);

    QLineEdit *meanErrorLabel = new QLineEdit(&resultsDialog);
    meanErrorLabel->setReadOnly(true);
    meanErrorLabel->setText(QString("%1").arg(mean));
    resultsDialogForm.addRow(tr("Mean error"), meanErrorLabel);

    QLineEdit *stddevErrorLabel = new QLineEdit(&resultsDialog);
    stddevErrorLabel->setReadOnly(true);
    stddevErrorLabel->setText(QString("%1").arg(stddev));
    resultsDialogForm.addRow(tr("STD dev"), stddevErrorLabel);

    QCheckBox *showPlaneCheckBox = new QCheckBox(&resultsDialog);
    showPlaneCheckBox->setChecked(false);
    resultsDialogForm.addRow(tr("Display fitted plane"), showPlaneCheckBox);

    QCheckBox *showFitErrorCheckBox = new QCheckBox(&resultsDialog);
    showFitErrorCheckBox->setChecked(true);
    resultsDialogForm.addRow(tr("Colorize error"), showFitErrorCheckBox);

    QDoubleSpinBox *minErrorSpinBox = new QDoubleSpinBox(&resultsDialog);
    minErrorSpinBox->setRange(-10000, 10000);
    minErrorSpinBox->setSingleStep(0.1);
    minErrorSpinBox->setValue(-2.*stddev);
    resultsDialogForm.addRow(tr("Min. error"), minErrorSpinBox);

    QDoubleSpinBox *maxErrorSpinBox = new QDoubleSpinBox(&resultsDialog);
    maxErrorSpinBox->setRange(-10000, 10000);
    maxErrorSpinBox->setSingleStep(0.1);
    maxErrorSpinBox->setValue(2.*stddev);
    resultsDialogForm.addRow(tr("Max. error"), maxErrorSpinBox);

    /// Add some standard buttons (Cancel/Ok) at the bottom of the dialog
    QDialogButtonBox resultsButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, &resultsDialog);
    resultsDialogForm.addRow(&resultsButtonBox);
    QObject::connect(&resultsButtonBox, SIGNAL(accepted()), &resultsDialog, SLOT(accept()));
    QObject::connect(&resultsButtonBox, SIGNAL(rejected()), &resultsDialog, SLOT(reject()));

    /// Show the dialog as modal
    if (resultsDialog.exec() != QDialog::Accepted)
        return;

    if (showFitErrorCheckBox->isChecked()) {
        time.restart();

        float minError = minErrorSpinBox->value();
        float maxError = maxErrorSpinBox->value();
        //float rangeError = maxError - minError;
        //uint32_t pointColor;

        for (size_t i=0; i<m_pointCloud->pclPointCloud()->points.size(); ++i)
            m_pointCloud->pclPointCloud()->points[i].intensity = maxError;

        int i = 0;
        for (std::vector<int>::const_iterator it = planeInlierIndices->indices.begin();
                it != planeInlierIndices->indices.end();
                ++it, ++i) {

            float error = qMax(minError, qMin(maxError, errorsInliers[i]));

            /*qDebug() << "projection:" << proj
                     << "planeDistance" << planeDistance
                     << "error" << error;*/

            m_pointCloud->pclPointCloud()->points[*it].intensity = error;

            /*
            float value = (error - minError) / rangeError;

            ////////////////////////////////////////////////////////////////////
            /// Returns the appropriate color from the "jet" colormap
            ///
            /// Implementing a Continuous "Jet" Colormap Function in GLSL
            /// http://www.metastine.com/?p=7
            ////////////////////////////////////////////////////////////////////
            float valuex4 = clamp(value, 0.f, 1.f) * 4.f;
            float r = min(valuex4 - 1.5f, -valuex4 + 4.5f);
            float g = min(valuex4 - 0.5f, -valuex4 + 3.5f);
            float b = min(valuex4 + 0.5f, -valuex4 + 2.5f);

            unsigned char ur = (unsigned char) 255.f * clamp(r, 0.f, 1.f); //R
            unsigned char ug = (unsigned char) 255.f * clamp(g, 0.f, 1.f); //G
            unsigned char ub = (unsigned char) 255.f * clamp(b, 0.f, 1.f); //B

            //qDebug() << error << value << r << g << b << ur << ug << ub;

            pointColor = (  static_cast<uint32_t>(ur) << 16 |
                            static_cast<uint32_t>(ug) <<  8 |
                            static_cast<uint32_t>(ub)       );

            m_pointCloud->points[*it].rgb = *reinterpret_cast<float*>(&pointColor);*/
        }

        qDebug() << "plane error colormap:" << time.elapsed() << "msecs";

        /// display point cloud
        //m_pclViewer->updatePointCloud(m_pointCloud, "cloud");
        m_pclViewer->removePointCloud(qPrintable(m_pointCloud->id()));
        //pcl::visualization::PointCloudColorHandlerGenericField<PointType> rgb(m_pointCloud, "intensity");
        m_pclViewer->addPointCloud<PointType>(m_pointCloud->pclPointCloud(),
                                              *m_pointCloud->colorHandler(),
                                              qPrintable(m_pointCloud->id()));
    }


    if (showPlaneCheckBox->isChecked()) {
        QString shapeId = QString("%1__fittedPlane").arg(m_pointCloud->id());
        m_pclViewer->removeShape(qPrintable(shapeId));
        m_pclViewer->addPlane(*planeCoefficients, qPrintable(shapeId));
    }
}

void ZCloudView::on_actionViewNormals_toggled(bool arg1)
{
    QString id = QString("%1__%2").arg(m_pointCloud->id()).arg("normals");
    if (arg1) {
        if (m_pointCloud->pclSurfaceElements()->size())
            m_pclViewer->addPointCloudNormals<SurfelType>(m_pointCloud->pclSurfaceElements(), 50, 1., qPrintable(id));
    } else {
        m_pclViewer->removePointCloud(qPrintable(id));
    }

    ui->qvtkWidget->update();
}

void ZCloudView::on_actionViewPoints_toggled(bool arg1)
{
    if (arg1) {
        m_pclViewer->addPointCloud<PointType>(m_pointCloud->pclPointCloud(),
                                              *m_pointCloud->colorHandler(),
                                              qPrintable(m_pointCloud->id()));
    } else {
        m_pclViewer->removePointCloud(qPrintable(m_pointCloud->id()));
    }

    ui->qvtkWidget->update();
}

void ZCloudView::on_actionViewSurface_toggled(bool arg1)
{
    QString id = QString("%1__%2").arg(m_pointCloud->id()).arg("surface");
    if (arg1) {
        m_pclViewer->addPolygonMesh(*m_pointCloud->pclSurfaceMesh(),
                                    qPrintable(id));
    } else {
        m_pclViewer->removePolygonMesh(qPrintable(id));
    }

    ui->qvtkWidget->update();
}

void ZCloudView::on_actionChangeBackgroundColor_triggered()
{
    QColor color = QColorDialog::getColor(m_currentBackgroundColor, this, "Select background color");
    if (color.isValid()) {
        m_currentBackgroundColor = color;
        m_pclViewer->setBackgroundColor(m_currentBackgroundColor.redF(),
                                        m_currentBackgroundColor.greenF(),
                                        m_currentBackgroundColor.blueF());
    }
}

void ZCloudView::on_actionRenderShadingFlat_triggered()
{
    ui->actionRenderShadingFlat->setChecked(true);
    ui->actionRenderShadingGouraud->setChecked(false);
    ui->actionRenderShadingPhong->setChecked(false);

#if PCL_VERSION_COMPARE(>=,1,7,0)
    QString id = QString("%1__%2").arg(m_pointCloud->id()).arg("surface");

    m_pclViewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_SHADING,
                pcl::visualization::PCL_VISUALIZER_SHADING_FLAT,
                qPrintable(id));
#endif
}

void ZCloudView::on_actionRenderShadingGouraud_triggered()
{
    ui->actionRenderShadingFlat->setChecked(false);
    ui->actionRenderShadingGouraud->setChecked(true);
    ui->actionRenderShadingPhong->setChecked(false);

#if PCL_VERSION_COMPARE(>=,1,7,0)
    QString id = QString("%1__%2").arg(m_pointCloud->id()).arg("surface");

    m_pclViewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_SHADING,
                pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD,
                qPrintable(id));
#endif
}

void ZCloudView::on_actionRenderShadingPhong_triggered()
{
    ui->actionRenderShadingFlat->setChecked(false);
    ui->actionRenderShadingGouraud->setChecked(false);
    ui->actionRenderShadingPhong->setChecked(true);

#if PCL_VERSION_COMPARE(>=,1,7,0)
    QString id = QString("%1__%2").arg(m_pointCloud->id()).arg("surface");

    m_pclViewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_SHADING,
                pcl::visualization::PCL_VISUALIZER_SHADING_PHONG,
                qPrintable(id));
#endif
}

void ZCloudView::on_actionRenderToFile_triggered()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save screenshot as..."),
                                                    ".",
                                                    tr("PNG image (*.png)"));

    if (fileName.isEmpty() || fileName.isNull())
        return;

    m_pclViewer->saveScreenshot(qPrintable(fileName));
}

void ZCloudView::on_actionToolsDelaunay2D_triggered()
{
    try {
        QTime time;
        time.start();

        cv::Subdiv2D subdiv(cv::Rect(-1000,-1000,2000,2000));

        const std::vector<PointType, Eigen::aligned_allocator<PointType> > &cloudPoints = m_pointCloud->pclPointCloud()->points;
        std::vector<PointType, Eigen::aligned_allocator<PointType> >::const_iterator it = cloudPoints.begin();
        std::vector<PointType, Eigen::aligned_allocator<PointType> >::const_iterator itEnd = cloudPoints.end();

        //std::vector<cv::Point2f> points;
        //points.reserve(cloudPoints.size());

        for ( ; it != itEnd; ++it) {
            const PointType &p = *it;
            //points.push_back(cv::Point2f(p.x, p.y));

            //qDebug() << "inserting" << p.x << p.y;

            subdiv.insert(cv::Point2f(p.x, p.y));
        }

        //subdiv.insert(points);

        std::vector<cv::Vec6f> triangleList;
        subdiv.getTriangleList(triangleList);

        //vector<cv::Point> pt(3);

        pcl::PolygonMesh::Ptr mesh = m_pointCloud->pclSurfaceMesh();
        std::vector< pcl::Vertices> &polygons = mesh->polygons;
        polygons.clear();


//        for( size_t i = 0; i < triangleList.size(); i++ ) {
//            const cv::Vec6f &t = triangleList[i];
//            pcl::Vertices vtx;
//            subdiv.getVertex()
//            vtx.vertices.push_back();
//            polygons.push_back();
//            /*pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
//            pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
//            pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
//            line(img, pt[0], pt[1], delaunay_color, 1, LINE_AA, 0);
//            line(img, pt[1], pt[2], delaunay_color, 1, LINE_AA, 0);
//            line(img, pt[2], pt[0], delaunay_color, 1, LINE_AA, 0);*/
//        }

        qDebug() << "delaunay triangulation finished in" << time.elapsed()
                 << "msecs with" << triangleList.size() << "triangles";

    } catch (...) {
        qWarning() << Q_FUNC_INFO << "failed!";
    }
}

} // namespace Z3D
