#include "utils/debug/error.h"

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
// PCL
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
// Need to include the pcl ros utilities
#include <pcl_ros/point_cloud.h>
// AD
#include <utils/debug/debug.h>
#include <utils/time/timer.h>
#include <compressed_rounded_image_transport/CompressedRoundedImage.h>
#include <ad_core/data_types/geometry/point3.h>
#include <ad_core/data_types/geometry/Rect3.h>
#include <vision/utils/image_utils/color_utils.h>
// C
#include <map>

class ClusterDetector {
public:
  typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
  typedef geometry_utils::Rect3f Bbox;

  //! the maximum distance btw two points to put them in the same cluster (meters)
  static const double CLUSTER_TOLERANCE = 0.2; // 0.10;

  //! the minimum size of a cluster to be kept
  static const int MIN_CLUSTER_SIZE = 35; // 100;

  //////////////////////////////////////////////////////////////////////////////

  ClusterDetector() {
    _cloud_subscriber = n.subscribe<Cloud>
        ("/camera/depth/points_filtered",
         1,
         &ClusterDetector::compute_cluster_from_cloud,
         this);

    _marker_publisher = n.advertise<visualization_msgs::Marker>
        ("cluster_markers", 1);

    _tree = new pcl::KdTreeFLANN<pcl::PointXYZ>();
    _tree_ptr = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(_tree);

    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
    ros::spin();
  }

  //////////////////////////////////////////////////////////////////////////////

  void compute_cluster_from_cloud(const Cloud::ConstPtr& cloud_msg) {
    maggieDebug2("compute_cluster_from_cloud()");

    // Creating the KdTree object for the search method of the extraction
    Timer timer;
    _tree_ptr->setInputCloud (cloud_msg);
    _cluster_indices.clear();
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (CLUSTER_TOLERANCE);
    ec.setMinClusterSize (MIN_CLUSTER_SIZE);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (_tree_ptr);
    ec.setInputCloud( cloud_msg);
    ec.extract (_cluster_indices);
    _n_clusters = (int) _cluster_indices.size();
    maggieDebug2("euclidin clustering:%g ms, \tnumber of clusters:%i",
                 timer.getTimeMilliseconds(),
                 _n_clusters);

    // Write the planar inliers to disk
#if 0
    timer.reset();
    pcl::PCDWriter writer;
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = _cluster_indices.begin ();
         it != _cluster_indices.end ();
         ++it) {
      Cloud::Ptr cloud_cluster(new Cloud);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit
           != it->indices.end (); pit++) {
        cloud_cluster->points.push_back (cloud_msg->points[*pit]);
      } // end loop cluster point

      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      std::cout << "cloud representing the Cluster: " <<
                   cloud_cluster->points.size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      j++;
    } // end loop cluster index
#endif

    compute_bounding_boxes(cloud_msg);
  } // end compute_cluster_from_cloud();

  //////////////////////////////////////////////////////////////////////////////

  inline void compute_bounding_boxes(const Cloud::ConstPtr& cloud_msg) {
    maggieDebug2("compute_bounding_boxes()");

    Timer timer;

    // save the last bboxes
    //    _prev_bboxes.clear();
    //    std::copy(_bboxes.begin(), _bboxes.end(), _prev_bboxes.begin());
    _prev_bboxes = _bboxes;

    _bboxes.clear();
    int cluster_index = 0;

    // share the future location via a marker
    _marker_msg.type = visualization_msgs::Marker::LINE_LIST;
    _marker_msg.lifetime = ros::Duration(0);
    // The namespace (ns) and id are used to create a unique name
    // for this marker. If a marker message is received with the same
    // ns and id, the new marker will replace the old one.
    _marker_msg.ns = "clusters";
    _marker_msg.header.frame_id = "/odom";
    _marker_msg.scale.x = 0.03; // line width
    // prepair space for storing colors and points
    _marker_msg.colors.clear();
    _marker_msg.colors.reserve(24 * _n_clusters);
    _marker_msg.points.clear();
    _marker_msg.points.reserve(24 * _n_clusters);
    // we need to set a default color even though we don't use it
    _marker_msg.color.r = 1; _marker_msg.color.g = 1;
    _marker_msg.color.b = 1; _marker_msg.color.a = 1;

    // iterate on the clusters
    for (std::vector<pcl::PointIndices>::const_iterator cluster_it
         = _cluster_indices.begin();
         cluster_it != _cluster_indices.end();
         ++cluster_it) {

      _bboxes.insert( std::pair<int, Bbox> (cluster_index, Bbox() ));
      Bbox* box = &_bboxes[cluster_index];
      // iterate on the points of the cluster
      for (std::vector<int>::const_iterator pt_it = cluster_it->indices.begin ();
           pt_it != cluster_it->indices.end (); pt_it++) {
        // extend the bbox
        (*box).extendToInclude(cloud_msg->points[*pt_it]);
        // change the orientation from
        // PCL (x right y down z forward)
        // to
        // nestk orientation (x right y up z backward)
        //        (*box).extendToInclude(AD::Point3<float>(
        //                                 cloud_msg->points[*pt_it].z,
        //                                 -cloud_msg->points[*pt_it].x,
        //                                 -cloud_msg->points[*pt_it].y
        //                                 ));
      } // end loop cluster point
      ++cluster_index;
      // maggieDebug2("curr_box:%s", box->to_string().c_str());

      // create a color for the cluster and push it 12 times
      // (the number of vertices of the cube)
      std_msgs::ColorRGBA cluster_color;
      cluster_color.a = 1;
      color_utils::indexed_color_norm(
            cluster_color.r, cluster_color.g, cluster_color.b,
            cluster_index);
      //    maggiePrint("cluster_color:%g, %g, %g, %g",
      //                cluster_color.r, cluster_color.g,
      //                cluster_color.b, cluster_color.a);
      for (int vertex = 0; vertex < 24 /*12 */; ++vertex)
        _marker_msg.colors.push_back(cluster_color);

      // copy the geometric information
      box->queue_all_vertices
          <geometry_msgs::Point, std::vector<geometry_msgs::Point> >
          (_marker_msg.points);
    } // end loop cluster index

    //    maggieDebug2("%i points, %i colors",
    //                 marker_msg.points.size(), marker_msg.colors.size());
    _marker_publisher.publish(_marker_msg);
    timer.printTime("compute_bounding_boxes()");
  } // end compute_bounding_boxes()

  //////////////////////////////////////////////////////////////////////////////

private:

  /**
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node, and the last
  * NodeHandle destructed will close down the node.
  */
  ros::NodeHandle n;

  ros::Subscriber _cloud_subscriber;
  ros::Publisher _marker_publisher;

  // clustering
  pcl::KdTreeFLANN<pcl::PointXYZ>* _tree;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr _tree_ptr;
  std::vector<pcl::PointIndices> _cluster_indices;
  int _n_clusters; //!< the number of clusters
  // bboxes
  std::map<int, Bbox> _bboxes;
  std::map<int, Bbox> _prev_bboxes;
  visualization_msgs::Marker _marker_msg;
}; // end class ClusterDetector

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  maggieDebug2("main()");
  ros::init(argc, argv, "blob_detection");

  ClusterDetector detec;

  return 0;
}


