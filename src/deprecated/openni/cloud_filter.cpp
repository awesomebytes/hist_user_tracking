
#include "utils/debug/error.h"
#include "utils/time/timer.h"

// ROS
#include <tf/transform_listener.h>
// PCL
#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/impl>
#include <pcl/segmentation/extract_clusters.h>
// Need to include the pcl ros utilities
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

class CloudFilter {
public:
  typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

  //! the size of the leaves for the downsampling (meters)
  static const double LEAF_SIZE = 0.1; // 0.05;

  //! the max rate for the filter (Hz)
  static const int MAX_RATE = 3;

  CloudFilter()
    : transform_listener(ros::Duration(10, 0)),
      src_to_dst_transform_found(false)
  {
    _cloud_subscriber = n.subscribe<Cloud>
        (//"/camera/depth/points_drop",
         "/camera/depth/points",
         1,
         &CloudFilter::filter_cloud,
         this);

    _cloud_publisher = n.advertise<Cloud>(
          "/camera/depth/points_filtered", 1);

    _cloud_filtered_src_frame = new Cloud();
    //_cloud_filtered_ptr = Cloud::Ptr(_cloud_filtered_src_frame);

    ros::spin();
  } // end ctor

  ////////////////////////////////////////////////////////////////////////////

  void filter_cloud(const Cloud::ConstPtr& cloud_msg) {
    maggieDebug2("filter_cloud()");
    Timer timer;
    ros::Rate rate(MAX_RATE);

    // Create the filtering object: downsample the dataset
    // using a custom leaf size
    timer.reset();
    _cloud_filtered_src_frame->points.clear();
    _vg.setInputCloud (cloud_msg);
    _vg.setLeafSize (LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    _vg.filter (*_cloud_filtered_src_frame);
    Timer::Time time_filtering = timer.getTimeMilliseconds();
    //    maggiePrint("orig_stamp:%g, diff with now:%g",
    //                _cloud_filtered->header.stamp.toSec(),
    //                (ros::Time::now() - _cloud_filtered->header.stamp).toSec() );
    //    _cloud_filtered->header.stamp = ros::Time::now() - ros::Duration(3);
    //    maggiePrint("orig_stamp:%g, diff with now:%g",
    //                _cloud_filtered->header.stamp.toSec(),
    //                (ros::Time::now() - _cloud_filtered->header.stamp).toSec() );

    // convert the referential
    timer.reset();
    //std::string dst_frame = "/odom";
    std::string dst_frame = "/base_link";

    std::string src_frame = _cloud_filtered_src_frame->header.frame_id;
    if (!src_to_dst_transform_found) {
      maggieDebug2("Waiting for the tf '%s'->'%s'",
                   src_frame.c_str(), dst_frame.c_str());
      std::string error_msg;
      bool tf_received =
          transform_listener.waitForTransform
          (dst_frame, src_frame, ros::Time::now(), ros::Duration(5.f),
           ros::Duration(0.01), &error_msg);
      if (tf_received) {
        maggieDebug1("tf '%s'->'%s' waited and successfully received.",
                     src_frame.c_str(), dst_frame.c_str());
        transform_listener.lookupTransform(dst_frame, src_frame,
                                           ros::Time(),
                                           src_to_dst_transform);
        src_to_dst_transform_found = true;
      }
      else {
        maggieDebug1("tf '%s'->'%s' waited, returned error '%s'. Aborting.",
                     src_frame.c_str(), dst_frame.c_str(), error_msg.c_str());
        return;
      } // end if tf_received
    } // end if not src_to_dst_transform_found

    //    std::string src_frame = _cloud_filtered->header.frame_id;
    //     // check if we can convert
    //    if (!transform_listener.canTransform
    //        (dst_frame, src_frame, ros::Time::now())) {
    //      maggieDebug2("Cannot convert '%s'->'%s'",
    //                   src_frame.c_str(), dst_frame.c_str());
    //      return;
    //      maggieDebug2("Waiting for the tf '%s'->'%s'",
    //                   src_frame.c_str(), dst_frame.c_str());
    //      std::string error_msg;
    //      bool tf_received =
    //          transform_listener.waitForTransform
    //          (dst_frame, src_frame, ros::Time::now(), ros::Duration(1.f),
    //           ros::Duration(0.01), &error_msg);
    //      if (tf_received) {
    //        maggieDebug1("tf '%s'->'%s' waited and successfully received.",
    //                     src_frame.c_str(), dst_frame.c_str());
    //      }
    //      else {
    //        maggieDebug1("tf '%s'->'%s' waited, returned error '%s'. Aborting.",
    //                     src_frame.c_str(), dst_frame.c_str(), error_msg.c_str());
    //        return;
    //      } // end if tf_received
    //    } // end if !canTransform()

    // make the proper conversion
    //    bool conv_success = pcl_ros::transformPointCloud
    //        (dst_frame,
    //         *_cloud_filtered, *_cloud_filtered,
    //         transform_listener);
    //    if (!conv_success) {
    //      maggieDebug2("Error while transformPointCloud(). Aborting.");
    //    }
    _cloud_filtered_dst_frame.clear();
    _cloud_filtered_dst_frame.header.frame_id = dst_frame;
    pcl_ros::transformPointCloud(*_cloud_filtered_src_frame, _cloud_filtered_dst_frame,
                                 src_to_dst_transform);

    maggieDebug2("Filtering: %g ms - Converting: %g ms, "
                 "\tcloud before filtering has %i pts, "
                 "after filtering %i.",
                 time_filtering,
                 timer.getTimeMilliseconds(),
                 cloud_msg->points.size(),
                 _cloud_filtered_src_frame->points.size());

    // emit it
    if (_cloud_publisher.getNumSubscribers() > 0)
      _cloud_publisher.publish(*_cloud_filtered_src_frame);

    // wait
    rate.sleep();
  } // end filter_cloud()

  //////////////////////////////////////////////////////////////////////////////

private:
  /**
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node,
  * and the last
  * NodeHandle destructed will close down the node.
  */
  ros::NodeHandle n;

  // filtering
  pcl::VoxelGrid<pcl::PointXYZ> _vg;
  Cloud* _cloud_filtered_src_frame;
  Cloud _cloud_filtered_dst_frame;
  //Cloud::Ptr _cloud_filtered_ptr;
  ros::Subscriber _cloud_subscriber;
  ros::Publisher _cloud_publisher;


  // transformer
  tf::TransformListener transform_listener;
  tf::StampedTransform src_to_dst_transform;
  bool src_to_dst_transform_found;
}; // end class CloudFilter

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  maggieDebug2("main()");
  ros::init(argc, argv, "cloud_filter");
  CloudFilter filter;

  return 0;
}

