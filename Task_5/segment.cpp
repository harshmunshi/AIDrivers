// Mandatory PCL includes
#include <iostream>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace pcl;
using namespace std;

// Closest algorithm for this particular task is plane model segmentation, assuming 
// road is a plane

class Segmentation {
    public:
        // default constructor
        Segmentation(){}
        pcl::PointCloud<pcl::PointXYZ>::Ptr readCloud(string path) {
            pcl::PCDReader reader;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>)
            reader.read (path, *cloud);
            return cloud;
        }
};