#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newcloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

    double resol = strtod(argv[2], NULL);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;

    sor.setInputCloud (cloud);
    sor.setLeafSize (float(resol), float(resol), float(resol));
    sor.filter (*newcloud);

    pcl::io::savePCDFileASCII (argv[3], *newcloud);
    std::cerr << "Downsample from: " << argv[1] << "(" << cloud->points.size () << " points) to" << argv[3]
              << "(" << newcloud->size() << " points)." << std::endl;

    return (0);
}
