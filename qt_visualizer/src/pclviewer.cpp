#include <ctime>
#include <glob.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <QFileDialog>
#include <QTimer>

#include "pclviewer.h"
#include "ui_pclviewer.h"

PCLViewer::PCLViewer(std::string dir, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLViewer)
{
    setupUI();
    setupCloudPointer();
    setupQVTKWindow();
    setupViewPorts();
    setupPlayTimer(42);
    bindEvents();
    loadIcons();
    setIcons();

    if (!dir.empty()) {
        std::cout << "Loading initial directory <" << dir << ">." << std::endl;
        loadPCDFilesFromDir(dir);
        loadCurrentPCD();
    }

    viewer->addPointCloud(cloud, "cloud");
    ui->qvtkWidget->update();
}

void PCLViewer::setupUI()
{
    ui->setupUi(this);
    this->setWindowTitle("PCL Viewer");
}

void PCLViewer::setupCloudPointer()
{
    cloud.reset(new PointCloudT);
}

void PCLViewer::setupQVTKWindow()
{
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

    /* An "interactor" can be set up for some additional interactive features;
     * press `h` in enabled interactor window to dump interaction help to the
     * terminal.
     */
    /*
     *viewer->setupInteractor(
     *    ui->qvtkWidget->GetInteractor(),
     *    ui->qvtkWidget->GetRenderWindow()
     *);
     */
}

void PCLViewer::setupViewPorts()
{
    viewer->createViewPort(0.0, 0.0, 2./3, 1.0, viewPortIDs["main"]);
    viewer->createViewPort(2./3, 0.5, 1.0, 1.0, viewPortIDs["fpv"]);
    viewer->createViewPort(2./3, 0.0, 1.0, 0.5, viewPortIDs["topdown"]);
    viewer->addText("FPV", 10, 10, "fpv", viewPortIDs["fpv"]);
    viewer->addText("Top-down", 10, 10, "topdown", viewPortIDs["topdown"]);

    // This is necessary to decouple viewpoint cameras:
    // <http://www.pcl-users.org/PCLVisualizer-Switching-between-viewports-tp4024958p4031302.html>
    for (const auto &viewPortID : viewPortIDs)
        viewer->createViewPortCamera(viewPortID.second);

    // TODO: Currently using hardcoded parameters fitting the `opaque` sample
    // data set. Particularly top-down parameters are strongly data set
    // dependent. This could be read from configuration files in the respective
    // PCD file directories.
    setCameraFPV(viewPortIDs["fpv"]);
    setCameraTopDown(viewPortIDs["topdown"]);

    drawRangeCircles();

    //// This is no longer needed in "modern" PCL versions:
    //// <http://www.pcl-users.org/What-should-be-used-instead-the-deprecated-updateCamera-method-tp4024155.html>
    //viewer->updateCamera();
}

void PCLViewer::bindEvents()
{
    connect(
        ui->advanceFrameButton,
        SIGNAL(clicked()),
        this,
        SLOT(advanceFrameButtonClicked())
    );

    connect(
        ui->frameSlider,
        SIGNAL(valueChanged(int)),
        this,
        SLOT(frameSliderValueChanged(int))
    );

    connect(
        ui->pointSizeSlider,
        SIGNAL(valueChanged(int)),
        this,
        SLOT(pointSizeSliderValueChanged(int))
    );

    connect(
        ui->browsePCDDirButton,
        SIGNAL(clicked()),
        this,
        SLOT(browsePCDDirButtonClicked())
    );

    connect(
        ui->playPauseButton,
        SIGNAL(clicked()),
        this,
        SLOT(playPauseButtonClicked())
    );

    connect(
        ui->dumpCameraPositionButton,
        SIGNAL(clicked()),
        this,
        SLOT(dumpCameraPositionButtonClicked())
    );

    connect(
        ui->toFPVButton,
        SIGNAL(clicked()),
        this,
        SLOT(toFPVButtonClicked())
    );

    connect(
        ui->toTopDownButton,
        SIGNAL(clicked()),
        this,
        SLOT(toTopDownButtonClicked())
    );

    connect(
        ui->resetCamerasButton,
        SIGNAL(clicked()),
        this,
        SLOT(resetCamerasButtonClicked())
    );

    connect(playTimer, SIGNAL(timeout()), this, SLOT(playTimerTimeout()));
}

void PCLViewer::loadIcons()
{
    auto s = style();
    icons["dir"] = s->standardIcon(QStyle::SP_DirOpenIcon);
    icons["next"] = s->standardIcon(QStyle::SP_MediaSkipForward);
    icons["play"] = s->standardIcon(QStyle::SP_MediaPlay);
    icons["pause"] = s->standardIcon(QStyle::SP_MediaPause);
}

void PCLViewer::setIcons()
{
    ui->browsePCDDirButton->setIcon(icons["dir"]);
    ui->advanceFrameButton->setIcon(icons["next"]);
    ui->playPauseButton->setIcon(icons["play"]);
}

void PCLViewer::setupPlayTimer(const int ms)
{
    playTimer = new QTimer(this);
    playTimer->setInterval(ms);
}

void PCLViewer::loadPCDFilesFromDir(const std::string dir)
{
    glob_t globbuf;
    std::string PCDGlob = dir + "/*.pcd";
    int status = glob(PCDGlob.c_str(), 0, NULL, &globbuf);
    if (status != 0) {
        std::cerr << "Could not load input files: no match for <"
            << PCDGlob << ">." << std::endl;
        return;
    }

    PCDFiles.assign(globbuf.gl_pathv, globbuf.gl_pathv + globbuf.gl_pathc);
    globfree(&globbuf);

    PCDDir = dir;
    PCDIterator = PCDFiles.cbegin();

    ui->frameSlider->setMaximum(PCDFiles.size());
    ui->frameSlider->setEnabled(true);
    ui->advanceFrameButton->setEnabled(true);
    ui->playPauseButton->setEnabled(true);
    ui->PCDDirLabel->setText(QString::fromStdString(PCDDir));
}

void PCLViewer::loadCurrentPCD()
{
    pcl::io::loadPCDFile<PointT>(*PCDIterator, *cloud);
    viewer->updatePointCloud(cloud, "cloud");
}

void PCLViewer::playTimerTimeout()
{
    advanceFrame();
}

void PCLViewer::advanceFrameButtonClicked()
{
    advanceFrame();
}

void PCLViewer::pointSizeSliderValueChanged(const int value)
{
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value
    );
    ui->qvtkWidget->update();
}

void PCLViewer::frameSliderValueChanged(const int value)
{
    if (advancing)
        return;

    PCDIterator = PCDFiles.cbegin() + value - 1;
    loadCurrentPCD();
    ui->qvtkWidget->update();
}

void PCLViewer::browsePCDDirButtonClicked()
{
    QString chosenDir = QFileDialog::getExistingDirectory(
        this,
        "Choose PCD directory",
        QString::fromStdString(PCDDir),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
    );
    loadPCDFilesFromDir(chosenDir.toStdString());
    loadCurrentPCD();
    ui->PCDDirLabel->setText(chosenDir);
    ui->qvtkWidget->update();
}

void PCLViewer::playPauseButtonClicked()
{
    playPause();
}

void PCLViewer::dumpCameraPositionButtonClicked()
{
    dumpCameraPosition();
}

void PCLViewer::toFPVButtonClicked()
{
    setCameraFPV(viewPortIDs["main"]);
}

void PCLViewer::toTopDownButtonClicked()
{
    setCameraTopDown(viewPortIDs["main"]);
}

void PCLViewer::resetCamerasButtonClicked()
{
    resetCameras();
}

void PCLViewer::advanceFrame()
{
    advancing = true;
    if (++PCDIterator == PCDFiles.cend())
        PCDIterator = PCDFiles.cbegin();
    loadCurrentPCD();

    int index = PCDIterator - PCDFiles.cbegin() + 1;
    ui->frameSlider->setValue(index);
    ui->frameLCDNumber->display(index);

    ui->qvtkWidget->update();
    advancing = false;
}

void PCLViewer::playPause()
{
    if (playTimer->isActive()) {
        playTimer->stop();
        ui->playPauseButton->setText("Play");
        ui->playPauseButton->setIcon(icons["play"]);
    } else {
        playTimer->start();
        ui->playPauseButton->setText("Pause");
        ui->playPauseButton->setIcon(icons["pause"]);
    }
}

void PCLViewer::setCameraFPV(const int viewPortID)
{
    viewer->setCameraPosition(0, 0, 0, 0, 0, -1, 0, 1, 0, viewPortID);
}

void PCLViewer::setCameraTopDown(const int viewPortID)
{
    viewer->setCameraPosition(0, 7, -2.7, 0, 0, -2.7, 0, 0, -1, viewPortID);
}

void PCLViewer::resetCameras()
{
    setCameraFPV(viewPortIDs["fpv"]);
    setCameraTopDown(viewPortIDs["topdown"]);
}

void PCLViewer::dumpCameraPosition()
{
    std::vector<pcl::visualization::Camera> camera;
    viewer->getCameras(camera);

    // Note that the member variable `view` corresponds to what is otherwise
    // in the API referred to as "up view", and `focal` corresponds to what is
    // otherwise referred to as "view direction".
    for (const auto &viewPortID : viewPortIDs) {
        const auto &c = camera[viewPortID.second];
        std::cout << "Current camera parameters for viewport <" << viewPortID.first << ">: " << std::endl
            << " - pos:      (" << c.pos[0]   << ", " << c.pos[1]   << ", " << c.pos[2]   << ")" << std::endl 
            << " - view dir: (" << c.focal[0] << ", " << c.focal[1] << ", " << c.focal[2] << ")" << std::endl
            << " - up view:  (" << c.view[0]  << ", " << c.view[1]  << ", " << c.view[2]  << ")" << std::endl; 
    }
}

void PCLViewer::drawRangeCircles()
{
    for (auto i = 1; i <= 5; ++i)
        drawRangeCircle(i);
}

void PCLViewer::drawRangeCircle(const float radius)
{
    pcl::ModelCoefficients cylinder;
    // A "small" value; we really want a circle, but `addCircle` does not allow
    // orientation changes. A cylinder with a minimal height emulates well.
    const float cylinderHeight = .001;

    // Coefficients given as: {x, y, z, dirx, diry, dirz, radius};
    cylinder.values = {
        0, -cylinderHeight / 2, 0,
        0,  cylinderHeight / 2, 0,
        radius
    };
    viewer->addCylinder(
        cylinder,
        "cylinder_" + std::to_string(radius),
        viewPortIDs["topdown"]
    );
}

PCLViewer::~PCLViewer()
{
    delete ui;
}
