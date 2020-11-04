// Mandatory PCL includes
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

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
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            reader.read (path, *cloud);
            return cloud;
        }

        void PrintProperties(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
            cout << "The cloud has " << cloud->size() << " points" << endl;
        }

        void segment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
            // pass through filter settings
            pcl::IndicesPtr indices (new std::vector <int>);
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.0, 1.0);
            pass.filter (*indices);

            // MinCutBased segmentation
            pcl::MinCutSegmentation<pcl::PointXYZ> seg;
            seg.setInputCloud (cloud);
            seg.setIndices (indices);

            pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
            pcl::PointXYZ point;
            point.x = 68.97;
            point.y = -18.55;
            point.z = 0.57;
            foreground_points->points.push_back(point);
            seg.setForegroundPoints (foreground_points);

            seg.setSigma (0.25);
            seg.setRadius (3.0433856);
            seg.setNumberOfNeighbours (14);
            seg.setSourceWeight (0.8);

            std::vector <pcl::PointIndices> clusters;
            seg.extract (clusters);

            std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

            // writing the pcd
            // pcl::PCDWriter writer;
            // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
            // std::stringstream ss;
            // ss << "cloud_cluster_segmented.pcd";
            // writer.write<pcl::PointXYZ> (ss.str(), *colored_cloud, false);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud();
        }

};

int main(int argc, char** argv) {
    string path = "1562926017.pcd";
    Segmentation seg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = seg.readCloud(path);
    seg.PrintProperties(cloud);
    seg.segment(cloud);
    cout << "Clusters made" << endl;
    return 0;
}