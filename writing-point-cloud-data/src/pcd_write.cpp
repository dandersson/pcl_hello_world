#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const std::string path = "test_pcd.pcd";

    // Fill in the cloud data
    cloud.width    = 8;
    cloud.height   = 4;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (auto &cp : cloud.points) {
        cp.x = 1024 * rand() / (RAND_MAX + 1.0f);
        cp.y = 1024 * rand() / (RAND_MAX + 1.0f);
        cp.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII(path, cloud);
    std::cerr
        << "Saved " << cloud.points.size() << " data points to <"
        << path << ">:" << std::endl;

    for (const auto &cp : cloud.points)
        std::cerr << "    " << cp.x << " " << cp.y << " " << cp.z << std::endl;
}
