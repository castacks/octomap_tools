#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <pcl/filters/passthrough.h>
#include <fstream>

int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr newcloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  std::string axis = argv[2];

  double setMin = strtod(argv[3], NULL);
  double setMax = strtod(argv[4], NULL);

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (axis);
  pass.setFilterLimits (setMin, setMax);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*newcloud);

  pcl::io::savePCDFileASCII (argv[5], *newcloud);
  std::cerr << "Saved " << newcloud->points.size () << " data points to test_pcd.pcd." << std::endl;

  return (0);
}
