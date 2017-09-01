#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

typedef pcl::PointXYZ              PointT;

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << std::endl;

  double resolution = strtod(argv[2], NULL);
  std::cout << "Octomap resolution is: " << resolution << std::endl;
  octomap::ColorOcTree tree(resolution);
  octomap::point3d sensor_origin(0.0, 0.0, 0.0);
  octomap::pose6d frame_origin(0.0, 0, 0, 0, 0, 0);
  octomap::Pointcloud octomap_cloud;

  for (unsigned int pt_idx = 0; pt_idx < cloud->points.size(); ++pt_idx)
  {
    const PointT& p = cloud->points[pt_idx];
    if (!std::isnan(p.z))
      octomap_cloud.push_back(p.x, p.y, p.z);
  }

  // insert scan (only xyz considered, no colors)
  tree.insertPointCloud(octomap_cloud, sensor_origin, frame_origin);

  tree.updateInnerOccupancy();

  bool result;
  result = tree.writeBinary(argv[3]); //.bt
  if (result)
	  std::cout << "Point cloud to octomap conversion is completed!" << std::endl;
  else
	  std::cout << "Conversion failed!" << std::endl;
  return (0);
}
