//////////////////////////////////////////////////////////////////////////
// PCLPlanes.cpp               
// This file is responsible for subscribing to the final mapped topics
// and generating a list of relevant planes 
//////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
// PCL-specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Segmentation-specific includes
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>

// Export to custom message type
#include "drone_ros_msgs/PlanesInliers.h"
#include "drone_ros_msgs/PlanesInliersArr.h"

ros::Publisher planepub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for PCL2, PCL and temp point clouds for filtering data
    pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;     
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Convert from msg to PCL2 to PCL
    pcl_conversions::toPCL(*cloud_msg, *cloud2);
    pcl::fromPCLPointCloud2(*cloud2, *cloud);
    std::cout << "Original Cloud Size: " << cloud->size() << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.01);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  drone_ros_msgs::PlanesInliersArr planes;

  int ii = 0, nr_points = (int) cloud->size ();
  // While 30% of the original cloud is still there
  while (cloud->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    // std::cout << "Plane " << ii << ": " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
    //                                     << coefficients->values[1] << " "
    //                                     << coefficients->values[2] << " " 
    //                                     << coefficients->values[3] << std::endl;
    // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    // Send result out on custom topic
    drone_ros_msgs::PlanesInliers tempplane;
    tempplane.a.data = coefficients->values[0];
    tempplane.b.data = coefficients->values[1];
    tempplane.c.data = coefficients->values[2];
    tempplane.d.data = coefficients->values[3];

    for(int jj=0; jj<(int) inliers->indices.size(); jj++){
        // tempplane.x.data.push_back(cloud_p->points[jj].x);
        tempplane.x.data.push_back(cloud_p->points[jj].x);
        tempplane.y.data.push_back(cloud_p->points[jj].y);
        tempplane.z.data.push_back(cloud_p->points[jj].z);
    }

    // Publish the data.
    planes.planes.push_back(tempplane);
    

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud.swap (cloud_f);
    ii++;
  }

  planepub.publish(planes);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_plane_detect");
  ros::NodeHandle nh;

//   Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/filtered_cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  planepub = nh.advertise<drone_ros_msgs::PlanesInliersArr> ("/pclplanes", 1);

  // Spin
  ros::spin ();
}