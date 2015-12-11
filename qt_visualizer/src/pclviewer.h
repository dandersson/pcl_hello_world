#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <QMainWindow>
#include <QTimer>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

using std::map;
using std::shared_ptr;
using std::string;
using std::vector;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
    class PCLViewer;
}

class PCLViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCLViewer(string dir, QWidget *parent = 0);
    ~PCLViewer();

public slots:
    void advanceFrameButtonClicked();
    void browsePCDDirButtonClicked();
    void dumpCameraPositionButtonClicked();
    void frameSliderValueChanged(const int value);
    void resetCamerasButtonClicked();
    void toFPVButtonClicked();
    void toTopDownButtonClicked();
    void playPauseButtonClicked();
    void playTimerTimeout();
    void pointSizeSliderValueChanged(const int value);

private:
    PointCloudT::Ptr cloud;
    QTimer *playTimer;
    Ui::PCLViewer *ui;
    bool advancing;
    map<string, QIcon> icons;
    shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    string PCDDir;
    map<string, int> viewPortIDs;
    vector<string> PCDFiles;
    vector<string>::const_iterator PCDIterator;
    void loadIcons();
    void loadPCDFilesFromDir(const string dir);

    void advanceFrame();
    void bindEvents();
    void drawRangeCircle(const float radius);
    void drawRangeCircles();
    void dumpCameraPosition();
    void loadCurrentPCD();
    void playPause();
    void resetCameras();
    void setCameraFPV(const int viewPortID);
    void setCameraTopDown(const int viewPortID);
    void setIcons();
    void setupCloudPointer();
    void setupPlayTimer(const int ms);
    void setupQVTKWindow();
    void setupViewPorts();
    void setupUI();
};

#endif
