#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <Eigen/Dense>

using namespace Eigen;

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

  double roll = strtod(argv[2], NULL);
  double pitch = strtod(argv[3], NULL);
  double yaw = strtod(argv[4], NULL);
  
  double x = strtod(argv[5], NULL);
  double y = strtod(argv[6], NULL);
  double z = strtod(argv[7], NULL);

  Eigen::Quaternion<double> q;

  Eigen::AngleAxis<double> aaZ(yaw, Eigen::Vector3d::UnitZ());

  Eigen::AngleAxis<double> aaY(pitch, Eigen::Vector3d::UnitY());

  Eigen::AngleAxis<double> aaX(roll, Eigen::Vector3d::UnitX());

  q = aaZ * aaY * aaX;


  Eigen::Matrix3d rotationMatrix = q.matrix();

  Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
  tf.topLeftCorner(3,3) = rotationMatrix;
  tf(0,3) = x;
  tf(1,3) = y;
  tf(2,3) = z;

  pcl::transformPointCloud(*cloud,*newcloud,tf);

  pcl::io::savePCDFileASCII (argv[8], *newcloud);
  std::cerr << "Saved " << newcloud->points.size () << " data points to test_pcd.pcd." << std::endl;

  return (0);
}
