#include "utils/debug/error.h"

#include <ros/ros.h>

#if 0
    /*
     * extract the planar components of the point cloud
     * until its size is small enough
     */
    timer.reset();
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    Cloud::Ptr cloud_plane(new Cloud ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    //int i=0;
    int nr_points = (int) _cloud_filtered->points.size ();

    // make the real extraction
    while (_cloud_filtered->points.size () > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(_cloud_filtered);
      seg.segment (*inliers, *coefficients); //*
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset."
                  << std::endl;
        break;
      }
      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (_cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_plane); //*
      std::cout << "cloud representing the planar component: " <<
                   cloud_plane->points.size () << " data points." << std::endl;
      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (_cloud_filtered); //*
    }
    timer.printTime("planar extraction");
#endif

int main(int argc, char** argv) {
  maggieDebug2("main()");
  ros::init(argc, argv, "FloorRemoval");
  maggiePrint("Nothing to do...");
  return 0;
}


