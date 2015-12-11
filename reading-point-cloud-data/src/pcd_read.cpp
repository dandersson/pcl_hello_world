#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    auto cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const auto path = "../../writing-point-cloud-data/build/test_pcd.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
        PCL_ERROR("Could not read file!\n");
        return -1;
    }

    std::cout
        << "Loaded " << cloud->points.size() << " data points from <" << path
        << "> with the following fields:" << std::endl;

    for (const auto &cp : cloud->points)
        std::cout << "    " << cp.x << " " << cp.y << " " << cp.z << std::endl;
}
