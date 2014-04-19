#include "utils/debug/error.h"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
// opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
// AD
#include <utils/debug/debug.h>
#include <utils/time/timer.h>
#include <vision/utils/connected_comp/disjoint_sets2.h>
#include <vision/utils/image_utils/border_remover.h>
#include <vision/utils/image_utils/color_utils.h>
#include <vision/utils/image_utils/drawing_utils.h>
#include <vision/utils/image_utils/value_remover.h>
#include "cv_conversion_float_uchar.h"
#include "connected_components_matcher.h"
#include "fast_cluster_functions.h"
#include "ros_utils/marker_utils.h"

//#define TIMER_ON

#define TRACK_BAR_SCALE_FACTOR 25.f

class FastClusterDetector {
public:
  //! true to show windows
  static const bool DISPLAY = true;
  //! true to publish the GUI image on the topic /fast_cluster_detector_img_gui
  static const bool PUBLISH_GUI_IMG = false;
  //! true to publish the objects names image on the topic fast_cluster_functions::OBJECTS_TOPIC_NAME
  static const bool PUBLISH_OBJECTS_IMG = true;
  //! true to write on disk images
  static const bool SAVE_IMAGES = false;

  ////////////////////////////////////////////////////////////////////////////////

  enum NaNRemovalMethod {
    NAN_REMOVAL_METHOD_NULL = 0,
    NAN_REMOVAL_METHOD_INPAINT = 1,
    NAN_REMOVAL_METHOD_AVERAGE_BORDER = 2
  }; // end NaNRemovalMethod
  static const NaNRemovalMethod NAN_REMOVAL_METHOD = NAN_REMOVAL_METHOD_NULL;

  //! the minimum size of a cluster to be kept in percents of the original image
  static const float MIN_CLUSTER_SIZE = 2E-3; // around 150 pixels for 320x240
  static const float MAX_CLUSTER_SIZE = .5; // half of the image

  static const double CANNY_PARAM1_DEFAULT_VAL = 1.3; // 2.0
  static const double CANNY_PARAM2_DEFAULT_VAL = 2.0; // 3.2

  //! the size of the kernel used to try to close contours (pixels)
  static const int MORPH_OPEN_KERNEL_SIZE = 3;

  //! the time in seconds for declaring the object as lost when not detected
  static const float TRACKING_TIMEOUT = 2;

  //////////////////////////////////////////////////////////////////////////////

  FastClusterDetector() {
    // subscribe to the image message
    image_transport::ImageTransport it(nh);
    _depth_image_subscriber = it.subscribe
        ("/camera_depth_decompressed",
         1,
         &FastClusterDetector::image_callback,
         this);

    if (PUBLISH_GUI_IMG)
      _objects_img_gui_pub = it.advertise(fast_cluster_functions::GUI_TOPIC, 1);
    if (PUBLISH_OBJECTS_IMG)
      _objects_img_pub = it.advertise(fast_cluster_functions::OBJECTS_IMAGE_TOPIC, 1);
    _objects_nb_pub = nh.advertise<std_msgs::Int32>
        (fast_cluster_functions::OBJECTS_NB_TOPIC, 1);

    // reprojection
    _cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>
        ("/camera/depth/camera_info");
    _cam_model.fromCameraInfo(_cam_info);

    // tracking publishers and subscribers
    _tracked_object_name_subscriber = nh.subscribe
        (fast_cluster_functions::TRACKED_OBJECT_NAME_TOPIC, 1,
         &FastClusterDetector::tracked_object_name_callback, this);
    _tracked_object_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>
        (fast_cluster_functions::TRACKED_OBJECT_POSE_TOPIC, 1);
    _tracked_object_name = fast_cluster_functions::NO_OBJECT;

    // marker
    _marker_pub = nh.advertise<visualization_msgs::Marker>
        ("/tracking_marker", 1);
    marker_utils::make_header(_marker, visualization_msgs::Marker::SPHERE,
                              "fast_cluster_tracked_object",
                              0.2,   0, 0, 0, 1,  "/odom");


    // configure the GUI for Canny parameters
    if (DISPLAY)
      cv::namedWindow("img_gui");
    canny_tb1_value = CANNY_PARAM1_DEFAULT_VAL * TRACK_BAR_SCALE_FACTOR;
    canny_tb2_value = CANNY_PARAM2_DEFAULT_VAL *  TRACK_BAR_SCALE_FACTOR;
    if (DISPLAY) {
      cv::createTrackbar("canny_param1", "img_gui", &canny_tb1_value, 100);
      cv::createTrackbar("canny_param2", "img_gui", &canny_tb2_value, 100);
    } // end if (DISPLAY)
  }

  //////////////////////////////////////////////////////////////////////////////

  //! this function is called each time an image is received
  void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    //maggieDebug2("image_callback()");

    Timer timer;
    // conversion to cv::Mat
    // cf
    // http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    try {
      //_bridge_img_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      _bridge_img_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat & img_ptr = _bridge_img_ptr->image;
    //cv::imshow("img_ptr", img_ptr);

    //image_utils::print_random_pts_float<float>(img_ptr, 10);
    //image_utils::print_random_pts_int<uchar>(img_ptr, 10);
    // ...
    image_utils::convert_float_to_uchar(img_ptr, img_uchar, alpha_trans, beta_trans);
    //cv::imshow("img_uchar", img_uchar);

    //print_random_pts_int<uchar>(img_uchar, 10);
    //    maggieDebug2("nonZero:%f",
    //                 1.f * cv::countNonZero(img_uchar) / (img_uchar.cols * img_uchar.rows));

#ifdef TIMER_ON
    timer.printTime("after remapping");
#endif // TIMER_ON

    /*
     * remove NaN from input image
     */
    if (NAN_REMOVAL_METHOD == NAN_REMOVAL_METHOD_NULL) {
      // simple copy and shrink a bit the image
      //      cv::Rect roi = geometry_utils::shrink_rec
      //          (cv::Rect(0, 0, img_uchar.cols, img_uchar.rows), 0.95);
      //      img_uchar(roi).copyTo(img_uchar_with_no_nan);
      img_uchar.copyTo(img_uchar_with_no_nan);
      image_utils::remove_value<uchar>(img_uchar_with_no_nan, image_utils::NAN_UCHAR);
    }

    else if (NAN_REMOVAL_METHOD == NAN_REMOVAL_METHOD_INPAINT) {
      // make inpaint
      img_uchar.copyTo(inpaint_mask);
      cv::threshold(img_uchar, inpaint_mask, 0, 255, cv::THRESH_BINARY_INV);
      cv::inpaint(img_uchar, inpaint_mask, img_uchar_with_no_nan, 5, cv::INPAINT_NS);
    } // en if NAN_REMOVAL_METHOD_INPAINT

    if (NAN_REMOVAL_METHOD == NAN_REMOVAL_METHOD_AVERAGE_BORDER) {
      // remove border
      roi = image_utils::remove_border(img_uchar, img_uchar_with_no_nan,
                                       image_utils::NAN_UCHAR, 0.5);
      //img_uchar.copyTo(img_uchar_with_no_nan);

      // replace the zeros with their left neighbour
      //image_utils::remove_value<uchar>(img_uchar_with_no_nan, image_utils::NAN_UCHAR);
    } // end if NAN_REMOVAL_METHOD_AVERAGE_BORDER

    /*
     * edge detection
     */
    // canny
    canny_param1 = 1.f * canny_tb1_value / TRACK_BAR_SCALE_FACTOR;
    canny_param2 = 1.f * canny_tb2_value / TRACK_BAR_SCALE_FACTOR;
    maggieDebug3("canny_param1:%g, canny_param2:%g, alpha_trans:%g",
                 canny_param1, canny_param2, alpha_trans);
    cv::Canny(img_uchar_with_no_nan, edges,
              alpha_trans * canny_param1, alpha_trans * canny_param2);
    //    cv::Scalar mean = cv::mean(edges);
    //    cv::minMaxLoc(edges, &minVal, &maxVal);
    //    maggiePrint("min:%f, max:%f, mean:%f", minVal, maxVal, mean[0]);

    // harris corners
    //    cv::cornerHarris(img_ptr, edges, 3, 3, 0.01);
    //    print_random_pts<float>(edges, 10);

    //    double threshold= 0.0001;
    //    cv::threshold(edges, harrisCorners, threshold,255,cv::THRESH_BINARY);
    //    cv::imshow("harrisCorners", harrisCorners);

#ifdef TIMER_ON
    maggiePrint("Time after Canny (param1:%g, param2:%g):%g ms",
                canny_param1, canny_param2, timer.time());
#endif // TIMER_ON

    /* invert the edges */
    cv::threshold(edges, edges_inverted, 128, 255, cv::THRESH_BINARY_INV);
    // close borders
    //image_utils::close_borders(edges_inverted_with_nan, (uchar) 255);
    //    cv::morphologyEx(edges_inverted, edges_inverted_opened,
    //                     cv::MORPH_OPEN,
    //                     cv::Mat(MORPH_OPEN_KERNEL_SIZE, MORPH_OPEN_KERNEL_SIZE, CV_8U, 255));
    cv::erode(edges_inverted, edges_inverted_opened,
              cv::Mat(MORPH_OPEN_KERNEL_SIZE, MORPH_OPEN_KERNEL_SIZE, CV_8U, 255));
    //timer.printTime("after close_borders()");

    /*
     * combine canny with nan
     */
    cv::min(edges_inverted_opened, img_uchar, edges_inverted_opened_with_nan);
    cv::threshold(edges_inverted_opened_with_nan, edges_inverted_opened_with_nan,
                  image_utils::NAN_UCHAR, 255, cv::THRESH_BINARY);
#ifdef TIMER_ON
    timer.printTime("after cv::min()");
#endif // TIMER_ON
    /*
     * connected components
     */
    _set.process_image(edges_inverted_opened_with_nan);
    _set.get_connected_components(edges_inverted_opened_with_nan.cols,
                                  _components_pts, _bounding_boxes);
#ifdef TIMER_ON
    unsigned int n_clusters_before = _components_pts.size();
    timer.printTime("after connected components");
#endif // TIMER_ON

    // filter them by size
    unsigned int min_cluster_size_pixels =
        1.f * MIN_CLUSTER_SIZE * edges_inverted_opened.cols * edges_inverted_opened.rows;
    unsigned int max_cluster_size_pixels =
        1.f * MAX_CLUSTER_SIZE * edges_inverted_opened.cols * edges_inverted_opened.rows;
    for (unsigned int cluster_idx = 0;
         cluster_idx < _components_pts.size(); ++cluster_idx) {
      DisjointSets2::Comp* curr_comp = &(_components_pts.at(cluster_idx));
      uchar curr_val = img_uchar_with_no_nan.at<uchar>(curr_comp->front());
      //Bbox* curr_bbox = &(_bounding_boxes.at(cluster_idx));
      bool need_delete = false;
      // value = black -> remove
      if (curr_val == 0) // can be a image_utils::NAN_UCHAR or 0 from threshold
        need_delete = true;
      // min size test
      else if (curr_comp->size() < min_cluster_size_pixels)
        need_delete = true;
      // max test size
      else if (curr_comp->size() > max_cluster_size_pixels)
        need_delete = true;
      // other tests
      else {
        // no other test came to my mind :)
      } // end tests

      // now delete if needed
      if (need_delete) {
        _components_pts.erase(_components_pts.begin() + cluster_idx);
        _bounding_boxes.erase(_bounding_boxes.begin() + cluster_idx);
        // rewind the counter not to skip the next elt
        --cluster_idx;
        continue;
      } // end if need_delete
    } // end loop _components_pts
#ifdef TIMER_ON
    maggiePrint("Time after filtering:%g ms, filtering:%i -> %i clusters",
                timer.time(),  n_clusters_before, _components_pts.size());
#endif // TIMER_ON
    /*
     * Matching
     */
    matcher.set_new_data(_components_pts, _bounding_boxes);
    matcher.match();
#ifdef TIMER_ON
    timer.printTime("after matcher.match()");
#endif // TIMER_ON

    /*
     * tracking
     */
    track_object();
#ifdef TIMER_ON
    timer.printTime("after tracking()");
#endif // TIMER_ON

    make_gui();

    ROS_INFO_THROTTLE(1, "Time for image_callback(): %g ms", timer.getTimeMilliseconds());

  } // end image_callback();

  ////////////////////////////////////////////////////////////////////////////////

  /*!
   * \arg pt the point in the 2D image
   * Return the point in the camera frame
   */
  inline cv::Point3d reproject (const cv::Point2i & pt) const {
    cv::Point3d line_vec = _cam_model.projectPixelTo3dRay(pt);
    //maggiePrint("dist:%g", dist);
    cv::Point3d vec_bad_orien =
        line_vec // direction vector at pt
        * (_bridge_img_ptr->image.at<float>(pt.y, pt.x) // depth
           / geometry_utils::norm(line_vec) // direction vector norm at pt
           );
    //    ROS_INFO_THROTTLE(1, "(%i, %i) -> (%g, %g, %g)",
    //                      pt.x, pt.y, vec_bad_orien.x, vec_bad_orien.y, vec_bad_orien.z);
    //return vec_bad_orien;
    return cv::Point3d(vec_bad_orien.z, -vec_bad_orien.x, -vec_bad_orien.y);
  } // end reproject()

  ////////////////////////////////////////////////////////////////////////////////

  //! set the new tracked object
  void tracked_object_name_callback(const std_msgs::Int32ConstPtr& pt_msg) {
    _tracked_object_name = pt_msg->data;
    _tracked_object_last_time_seen.reset();
    ROS_INFO("Now tracking object %i", _tracked_object_name);
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! the function for tracking the object selected by the user.
      Reproject the wanted object and emit the point */
  inline void track_object() {
    if (_tracked_object_name == fast_cluster_functions::NO_OBJECT) {
      // emit a (0, 0, 0) pt
      geometry_msgs::PoseStamped msg;
      msg.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      msg.header.frame_id = "/odom";
      msg.header.stamp = ros::Time::now();
      _tracked_object_pose_publisher.publish(msg);
      // clear marker
      _marker.action = 2; // delete
      _marker_pub.publish(_marker);
      return;
    } // end if (_tracked_object_name != fast_cluster_functions::NO_OBJECT)

    // try to get the points
    const ConnectedComponentsMatcher::Comp *pts, *pts_resized;
    const ConnectedComponentsMatcher::Bbox* bbox;
    bool lookup_success = matcher.reco_history_lookup
        (_tracked_object_name, 0, pts, pts_resized, bbox);
    if (!lookup_success) {
      if (_tracked_object_last_time_seen.getTimeMilliseconds() / 1000
          > TRACKING_TIMEOUT) {
        maggiePrint("the tracked object %i has been lost for too long, stopping.",
                    _tracked_object_name);
        _tracked_object_name = fast_cluster_functions::NO_OBJECT;
        // clear marker
        _marker.action = 2; // delete
        _marker_pub.publish(_marker);
        return;
      }
      ROS_INFO_THROTTLE(1, "Impossible to find the tracked object %i (not found since %g ms)!",
                        _tracked_object_name, _tracked_object_last_time_seen.time());
      // put a red marker (error)
      _marker.color.r = 1; _marker.color.g = 0; _marker.action = 0; // add
      _marker_pub.publish(_marker);
      return;
    }
    _tracked_object_last_time_seen.reset();
    // reset the tracked object center
    _tracked_object_pose_cam_frame.header.frame_id = "/camera_link";
    _tracked_object_pose_cam_frame.header.stamp = ros::Time::now() - ros::Duration(1);
    _tracked_object_pose_cam_frame.pose.position.x = 0;
    _tracked_object_pose_cam_frame.pose.position.y = 0;
    _tracked_object_pose_cam_frame.pose.position.z = 0;
    _tracked_object_pose_cam_frame.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    // find the cluster average
    for (unsigned int curr_pt_idx = 0; curr_pt_idx < pts->size(); ++curr_pt_idx) {
      cv::Point3d curr_pt = reproject((*pts)[curr_pt_idx]);
      _tracked_object_pose_cam_frame.pose.position.x += curr_pt.x;
      _tracked_object_pose_cam_frame.pose.position.y += curr_pt.y;
      _tracked_object_pose_cam_frame.pose.position.z += curr_pt.z;
    } // end loop curr_pt_idx
    _tracked_object_pose_cam_frame.pose.position.x *= 1.f / pts->size();
    _tracked_object_pose_cam_frame.pose.position.y *= 1.f / pts->size();
    _tracked_object_pose_cam_frame.pose.position.z *= 1.f / pts->size();

    // convert it into odom frame
    try {
      _tf_listener.transformPose("/odom",  ros::Time(0),
                                 _tracked_object_pose_cam_frame,
                                 "/odom",
                                 _tracked_object_pose_odom_frame);
    } catch (tf::ExtrapolationException e) {
      ROS_WARN("transform error:'%s'", e.what());
      // put an orange marker (warning)
      _marker.color.r = 1; _marker.color.g = 0.5; _marker.action = 0; // add
      return;
    }

    ROS_INFO_THROTTLE(1, "The wanted object %i is in (%g, %g, %g) in /camera_link"
                      "= (%g, %g, %g) in /odom frame.",
                      _tracked_object_name,
                      _tracked_object_pose_cam_frame.pose.position.x,
                      _tracked_object_pose_cam_frame.pose.position.y,
                      _tracked_object_pose_cam_frame.pose.position.z,
                      _tracked_object_pose_odom_frame.pose.position.x,
                      _tracked_object_pose_odom_frame.pose.position.y,
                      _tracked_object_pose_odom_frame.pose.position.z);

    _tracked_object_pose_publisher.publish(_tracked_object_pose_odom_frame);
    // green color (OK)
    _marker.color.r = 0; _marker.color.g = 1; _marker.action = 0; // add
    //    _marker.header.frame_id = "/camera_link";
    //    _marker.pose.position = _tracked_object_pose_cam_frame.pose.position;
    _marker.pose.position = _tracked_object_pose_odom_frame.pose.position;
    _marker_pub.publish(_marker);
  } // end track_object();

  //////////////////////////////////////////////////////////////////////////////

  //! make a fancy interface
  inline void make_gui() {
#ifdef TIMER_ON
    Timer timer;
#endif // TIMER_ON

    // fill _objects_names with the names of the recognized objects
    if (SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG || PUBLISH_OBJECTS_IMG) {
      _objects_names.create(edges_inverted.size());
      _objects_names = fast_cluster_functions::NO_OBJECT;
      for (unsigned int cluster_idx = 0; cluster_idx < _components_pts.size(); ++cluster_idx) {
        ConnectedComponentsMatcher::ObjectName obj_name = 0;
        bool success = matcher.get_comp_name(0, cluster_idx, obj_name);
        if (!success)
          continue;
        DisjointSets2::Comp* curr_comp = &_components_pts.at(cluster_idx);
        for (unsigned int compt_pt_idx = 0; compt_pt_idx < curr_comp->size();
             ++compt_pt_idx) {
          _objects_names((*curr_comp)[compt_pt_idx]) = obj_name;
        } // end loop compt_pt_idx
      } // end loop cluster_idx
#ifdef TIMER_ON
      timer.printTime("filling of _objects_names");
#endif // TIMER_ON
    } // end if SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG || PUBLISH_OBJECTS_IMG


    // paint the matched components on _components_illus
    if (SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG) {
      fast_cluster_functions::paint_object_image(_objects_names, _components_illus);
      // put indices name on them
      for (unsigned int cluster_idx = 0; cluster_idx < _bounding_boxes.size(); ++cluster_idx) {
        ConnectedComponentsMatcher::ObjectName obj_name = 0;
        bool success = matcher.get_comp_name(0, cluster_idx, obj_name);
        if (!success)
          continue;
        cv::putText(_components_illus,
                    StringUtils::cast_to_string(obj_name),
                    .5 * (_bounding_boxes[cluster_idx].tl() + _bounding_boxes[cluster_idx].br()),
                    CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 255, 255), 2);
      } // end loop cluster_idx
#ifdef TIMER_ON
      timer.printTime("painting of _components_illus");
#endif // TIMER_ON
    } // end if SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG


    // make the collage on _img_gui
    if (SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG) {
      _img_gui.create(2 * img_uchar.rows, 3 * img_uchar.cols);
      _img_gui = cv::Vec3b(0, 0, 0);
      // 1st row
      cv::cvtColor(img_uchar, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      image_utils::paste_img(_img_gui_rgb_buffer, _img_gui,
                             0 * img_uchar.cols, 0 * img_uchar.rows, 0, "img_uchar");
      //#if NAN_REMOVAL_METHOD == NAN_REMOVAL_METHOD_INPAINT
      //    cv::cvtColor(inpaint_mask, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      //    image_utils::paste_img(img_gui_rgb_buffer, _img_gui,
      //                           1 * img_uchar.cols, 0 * img_uchar.rows);
      //#endif // NAN_REMOVAL_METHOD_INPAINT
      cv::cvtColor(img_uchar_with_no_nan, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      image_utils::paste_img(_img_gui_rgb_buffer, _img_gui,
                             1 * img_uchar.cols, 0 * img_uchar.rows, 0, "img_uchar_with_no_nan");
      cv::cvtColor(edges_inverted, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      image_utils::paste_img(_img_gui_rgb_buffer, _img_gui,
                             2 * img_uchar.cols, 0 * img_uchar.rows, 0, "edges_inverted");

      // 2nd row
      cv::cvtColor(edges_inverted_opened, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      image_utils::paste_img(_img_gui_rgb_buffer, _img_gui,
                             0 * img_uchar.cols, 1 * img_uchar.rows, 0, "edges_inverted_opened");
      cv::cvtColor(edges_inverted_opened_with_nan, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      image_utils::paste_img(_img_gui_rgb_buffer, _img_gui,
                             1 * img_uchar.cols, 1 * img_uchar.rows, 0, "edges_inverted_opened_with_nan");
      image_utils::paste_img(_components_illus, _img_gui,
                             2 * img_uchar.cols, 1 * img_uchar.rows, 0, "components_illus");
#ifdef TIMER_ON
      timer.printTime("collage in _img_gui");
#endif // TIMER_ON
    } // end if SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG


    if (SAVE_IMAGES) {
      cv::imwrite("0-img_gui.png", _img_gui);
      cv::imwrite("1-img_uchar.png", img_uchar);
      cv::imwrite("2-img_uchar_with_no_nan.png", img_uchar_with_no_nan);
      cv::imwrite("3-edges_inverted.png", edges_inverted);
      cv::imwrite("4-edges_inverted_opened.png", edges_inverted_opened);
      cv::imwrite("5-edges_inverted_opened_with_nan.png", edges_inverted_opened_with_nan);
      cv::imwrite("6-components_illus.png", _components_illus);
#ifdef TIMER_ON
      timer.printTime("image saving");
#endif // TIMER_ON
    } // end if (SAVE_IMAGES)


    if (PUBLISH_GUI_IMG) {
      cv_bridge::CvImage out_msg;
      out_msg.header.frame_id = "odom";
      out_msg.header.stamp    = ros::Time::now();
      out_msg.encoding        = sensor_msgs::image_encodings::BGR8;
      out_msg.image           = _img_gui;
      _objects_img_gui_pub.publish(out_msg.toImageMsg());
#ifdef TIMER_ON
      timer.printTime("_img_gui publishing");
#endif // TIMER_ON
    } // end if PUBLISH_GUI_IMG


    if (PUBLISH_OBJECTS_IMG) {
      cv_bridge::CvImage out_msg;
      out_msg.header.frame_id = "odom";
      out_msg.header.stamp    = ros::Time::now();
      out_msg.encoding        = sensor_msgs::image_encodings::TYPE_16SC1;
      out_msg.image           = _objects_names;
      _objects_img_pub.publish(out_msg.toImageMsg());
#ifdef TIMER_ON
      timer.printTime("_objects_names publishing");
#endif // TIMER_ON
    } // end if (PUBLISH_OBJECTS_IMG)

    // publish the number of objects
    std_msgs::Int32 objects_nb_pub_msg;
    objects_nb_pub_msg.data = matcher._recognition_history.size();
    _objects_nb_pub.publish(objects_nb_pub_msg);
    //ROS_INFO("Publishing _objects_nb:%i", objects_nb_pub_msg.data);

    if (DISPLAY) {
      cv::imshow("img_gui", _img_gui);
      int key_code = (char) cv::waitKey(1);
      if (key_code == 27)
        exit(0);
#ifdef TIMER_ON
      timer.printTime("after imshow()");
#endif // TIMER_ON
    } // end if (DISPLAY)

  } // end make_gui();

  //////////////////////////////////////////////////////////////////////////////

private:
  /**
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node, and the last
  * NodeHandle destructed will close down the node.
  */
  ros::NodeHandle nh;

  // float receiver
  cv_bridge::CvImageConstPtr _bridge_img_ptr;
  image_transport::Subscriber _depth_image_subscriber;

  // float -> uchar
  double alpha_trans;
  double beta_trans;
  cv::Mat1b img_uchar;

  // nan removal
#if NAN_REMOVAL_METHOD == NAN_REMOVAL_METHOD_INPAINT
  cv::Mat1b inpaint_mask;
#endif // NAN_REMOVAL_METHOD_INPAINT

#if NAN_REMOVAL_METHOD == NAN_REMOVAL_METHOD_AVERAGE_BORDER
  cv::Rect roi;
#endif // NAN_REMOVAL_METHOD_AVERAGE_BORDER
  cv::Mat1b img_uchar_with_no_nan;

  // edge detection
  double canny_param1;
  double canny_param2;
  int canny_tb1_value;
  int canny_tb2_value;

  cv::Mat1b edges;
  cv::Mat1b edges_inverted;
  cv::Mat1b edges_inverted_opened_with_nan;
  cv::Mat1b edges_inverted_opened;
  //cv::Mat1b harrisCorners;

  // connected comps
  DisjointSets2 _set;
  std::vector<std::vector<cv::Point> > _components_pts;
  std::vector<cv::Rect> _bounding_boxes;
  cv::Mat1i _objects_names; //!< will be filled with object names
  cv::Mat3b _components_illus;

  // clustering
  image_transport::Publisher _objects_img_pub;
  ConnectedComponentsMatcher matcher;
  ros::Publisher _objects_nb_pub;

  // tracking part
  ros::Subscriber _tracked_object_name_subscriber;
  ConnectedComponentsMatcher::ObjectName _tracked_object_name;
  ros::Publisher _tracked_object_pose_publisher;
  Timer _tracked_object_last_time_seen;

  /* reprojection and filtering */
  sensor_msgs::CameraInfoConstPtr _cam_info;
  image_geometry::PinholeCameraModel _cam_model;
  tf::TransformListener _tf_listener;

  // results
  geometry_msgs::PoseStamped _tracked_object_pose_cam_frame;
  geometry_msgs::PoseStamped _tracked_object_pose_odom_frame;
  visualization_msgs::Marker _marker;
  ros::Publisher _marker_pub;

  // GUI
  cv::Mat3b _img_gui;
  cv::Mat3b _img_gui_rgb_buffer;
  image_transport::Publisher _objects_img_gui_pub;
}; // end class FastClusterDetector

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  maggieDebug2("main()");
  ros::init(argc, argv, "FastClusterDetector");

  FastClusterDetector detec;

  /**
  * ros::spin() will enter a loop, pumping callbacks.  With this version, all
  * callbacks will be called from within this thread (the main one).  ros::spin()
  * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
  */
  ros::spin();

  return 0;
}
