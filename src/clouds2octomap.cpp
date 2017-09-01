#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <Eigen/Geometry>
#include <vector>
#include <map>

typedef pcl::PointXYZ                   Point;
typedef pcl::PointCloud<pcl::PointXYZ>  PointCloud;


bool readPCD(const char* pcd_file_name, PointCloud& cloud, double resolution) {
    PointCloud raw_cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file_name, raw_cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file .pcd \n");
      return false;
    }

    // voxel filter
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(raw_cloud.makeShared());
    sor.setLeafSize((float)resolution, (float)resolution, (float)resolution);
    sor.filter(cloud);

    return true;
}

bool readCSV(const char* csv_file_name, std::map<std::string, std::vector<double> >& data,
             std::vector<std::string>& names) {
    std::ifstream fid(csv_file_name);
    std::string line;

    while(std::getline(fid, line)) {
        std::vector<double> row;
        std::stringstream iss(line);
        std::string val;

        int idx = 0;
        std::string header;
        while (std::getline(iss, val, ',')) {
            if(idx == 0) {
                header = val;
                idx++;
                continue;
            }

            double v = std::atof(val.c_str());
            row.push_back(v);
            idx++;
        }
        names.push_back(header);
        data.insert(std::make_pair(header, row));
    }
}

bool insertCloud(octomap::ColorOcTree& tree, const PointCloud& cloud,
                 octomap::point3d sensor_origin,
                 octomap::pose6d frame_origin) {
    octomap::Pointcloud octomap_cloud;

    for (size_t pt_idx = 0; pt_idx < cloud.size(); ++pt_idx)
    {
      const Point& p = cloud[pt_idx];
      if (!std::isnan(p.z))
        octomap_cloud.push_back(p.x, p.y, p.z);
    }

    tree.insertPointCloud(octomap_cloud, sensor_origin, frame_origin);
}

int main (int argc, char** argv) {

    std::cout << "Joining Pointclouds to generate Octomap: " << std::endl;

    double resolution;
    char* octomap_file_name;
    char* tf_file_name;
    char* insert_clouds_path;
    std::map<std::string, std::vector<double> > transforms;
    std::vector<PointCloud> insert_clouds;
    std::vector<std::string> insert_cloud_names;

    resolution = strtod(argv[1], NULL);
    std::cout << "1. Octomap resolution:          " << resolution << std::endl;

    octomap_file_name = argv[2];

    tf_file_name = argv[3];
    readCSV(tf_file_name, transforms, insert_cloud_names);
    std::cout << "2. Number of loaded tfs:        " << transforms.size() << std::endl;

    insert_clouds_path = argv[4];
    int num_insert_clouds = int(insert_cloud_names.size());
    std::cout << "3. Number of insert clouds:     " << num_insert_clouds << std::endl;

    if(num_insert_clouds == 0 || transforms.size() == 0) {
        std::cout << "Error! no point cloud detected." << std::endl;
        return -1;
    }
    for(size_t i=0; i<num_insert_clouds; i++) {
        std::string pcd_name = insert_clouds_path +
                               std::string("_") +
                               insert_cloud_names[i] +
                               std::string("_reg.pcd");

        PointCloud insert_cloud;
        readPCD(pcd_name.c_str(), insert_cloud, resolution);
        insert_clouds.push_back(insert_cloud);

        std::cout << "   size of insert cloud " << i+1 << ":      " << insert_cloud.size() << std::endl;
    }

    // Create octree
    std::cout << "4. Adding pointclouds:       " << std::endl;
    octomap::ColorOcTree tree(resolution);
    octomap::point3d sensor_origin(0.0, 0.0, 0.0);

    for(int i=0; i<num_insert_clouds; i++) {
        std::vector<double> transform = transforms.at(insert_cloud_names[i]);
        Eigen::Matrix3d rotation;
        Eigen::Vector3d translation;
        rotation << transform[0], transform[1], transform[2],
                    transform[4], transform[5], transform[6],
                    transform[8], transform[9], transform[10];
        translation << transform[3], transform[7], transform[11];

        Eigen::Matrix4d T;
        T << rotation.transpose(), -rotation.transpose()*translation,
             0.0, 0.0, 0.0, 1.0;

        Eigen::Quaterniond quaternion(rotation);

        PointCloud insert_cloud_transformed;
        pcl::transformPointCloud(insert_clouds[i], insert_cloud_transformed,T);
        octomap::pose6d frame_origin(octomath::Vector3(translation[0], translation[1], translation[2]),
                                     octomath::Quaternion(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()));
        insertCloud(tree, insert_cloud_transformed, sensor_origin, frame_origin);
//        insertCloud(tree, insert_clouds[i], sensor_origin, frame_origin);
        std::cout << "   Added insert cloud " << i+1 << std::endl;
    }

    tree.updateInnerOccupancy();

    bool result;
    result = tree.writeBinary(octomap_file_name); //.bt
    if (result) {
        std::cout << "5. Saved .bt file to:           " << octomap_file_name << "(resolution: " <<resolution << ")" << std::endl;
        std::cout << "   To visualize:                " << "octovis " << octomap_file_name << std::endl;
    }
    else {
        std::cout << "5. Error! Failed to save .bt file!!!" << std::endl;
    }
    return 0;
}
