// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/camera_subscriber.h>
// opencv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// AD
#include <long_term_memory/ltm_path.h>
#include <utils/time/timer.h>
#include <utils/geom/geometry_utils.h>
//#include <vision/utils/image_utils/cv_conversion_float_uchar.h>
#include <vision/utils/image_utils/opencv_face_detector.h>
#include <ad_core/data_types/geometry/Rect3.h>

class FaceDetector {
public:

  // false positive discriminaton
  //! the number of depth samples
  static const unsigned int DEPTH_SAMPLE_SIZE = 10;
  //! the radius of a head, from the eye surface to the head center (m)
  static const double HEAD_DEPTH = 0.15;
  //! from forehead to chin / 2
  static const double HEAD_HEIGHT = 0.25;
  //! the maximum bounding-box depth of the face points (m)
  static const double MAX_SAMPLE_DEPTH = 0.08;
  //! the maximum bounding-box width of the face points (m)
  static const double MAX_SAMPLE_WIDTH = 0.2;

  FaceDetector()
    :  n(),
      // subscribe to the image message
      _rgb_image_subscriber(n, "/camera/rgb/image_color", 1),
      _depth_image_subscriber(n, "/camera/depth/image", 1)
  {
    // synchronize depth and rgb callbacks
    // ExactTime policy
    // cf http://www.ros.org/wiki/message_filters#Example_.28C.2B-.2B-.29-1
    //    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync
    //        (_rgb_image_subscriber, _depth_image_subscriber, 1);

    // ApproximateTime synchronizer
    typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    //policy.setMaxIntervalDuration (ros::Duration(30.f / 1000)); // max package of 30ms
    message_filters::Synchronizer<MySyncPolicy> sync
        (MySyncPolicy(1), _rgb_image_subscriber, _depth_image_subscriber);

    sync.registerCallback(boost::bind(&FaceDetector::image_callback, this, _1, _2));

    //project point cloud to image
    _cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>
        ("/camera/rgb/camera_info");
    _cam_model.fromCameraInfo(_cam_info);

    _marker_publisher = n.advertise<visualization_msgs::Marker>("cluster_markers", 10);

    cascadeClassifier = image_utils::create_face_classifier();

    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
    ros::spin();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! this function is called each time an image is received
  void image_callback(const sensor_msgs::ImageConstPtr& rgb_msg,
                      const sensor_msgs::ImageConstPtr& depth_msg) {
    maggieDebug2("image_callback()");

    Timer timer;
    // conversion to cv::Mat, cf
    // http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    try {
      bridge_rgb_img_ptr = cv_bridge::toCvShare
          (rgb_msg, sensor_msgs::image_encodings::BGR8);
      bridge_depth_img_ptr = cv_bridge::toCvShare(depth_msg);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    const cv::Mat & rgb_img = bridge_rgb_img_ptr->image;
    const cv::Mat & depth_img = bridge_depth_img_ptr->image;
    //timer.printTime("cv_bridge conversion");

    // do the stuff
    rgb_img.copyTo(img_out);

    // detect with opencv
    image_utils::detect_with_opencv(rgb_img, cascadeClassifier,
                                    small_img, faces_not_filtered);
    nb_faces = faces_not_filtered.size();
    timer.printTime("detect_with_opencv()");

    remove_outliers(rgb_img, depth_img);
    maggieDebug2("After filtering, %i faces: %s",
                 faces_centers_3d.size(),
                 StringUtils::iterable_to_string(faces_centers_3d).c_str());
    timer.printTime("remove_outliers()");

    // draw
    // removed faces in red
    for ( std::vector< cv::Rect >::const_iterator r = faces_not_filtered.begin();
          r != faces_not_filtered.end() ; ++r)
      cv::rectangle(img_out, *r, CV_RGB(255, 0, 0), 1);
    // kept faces in green
    for ( std::vector< cv::Rect >::const_iterator r = faces_filtered.begin();
          r != faces_filtered.end() ; ++r)
      cv::rectangle(img_out, *r, CV_RGB(0, 255, 0), 2);

    /*
     * now build the marker message
     */
    _marker_msg.type = visualization_msgs::Marker::SPHERE_LIST;
    _marker_msg.header.frame_id = "/openni_rgb_frame";
    _marker_msg.header.stamp = ros::Time();
    _marker_msg.ns = "heads";
    _marker_msg.id = 0;
    // color
    _marker_msg.color.g = 1;
    _marker_msg.color.a = 1;
    // position
    _marker_msg.scale.x = HEAD_DEPTH;
    _marker_msg.scale.y = HEAD_DEPTH;
    _marker_msg.scale.z = HEAD_HEIGHT;
    // points
    _marker_msg.points.clear();
    for (unsigned int pt_idx = 0; pt_idx < faces_centers_3d.size(); ++pt_idx) {
      _marker_msg.points.push_back(geometry_msgs::Point());
      _marker_msg.points.back().x = faces_centers_3d[pt_idx].x;
      _marker_msg.points.back().y = faces_centers_3d[pt_idx].y;
      _marker_msg.points.back().z = faces_centers_3d[pt_idx].z;
    } // end loop pt_idx
    //    maggieDebug2("_marker_msg.points: %s",
    //                 StringUtils::iterable_to_string(_marker_msg.points).c_str());

    // ...
    // emit it
    _marker_publisher.publish(_marker_msg);

    timer.printTime("image_callback()");

    //cv::imshow("rgb_img_ptr", rgb_img);
    cv::imshow("img_out", img_out);
    // convert the depth to an RGB image for display
    //    image_utils::convert_float_to_uchar(depth_img, depth_color_to_uchar,
    //                                        alpha_trans, beta_trans);
    //    cv::imshow("depth_color_to_uchar", depth_color_to_uchar);

    int key_code = (char) cv::waitKey(1);
    if (key_code == 27)
      exit(-1);

  } // end image_callback();

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * \arg pt the point in the 2D image
   */
  inline cv::Point3d reproject (const cv::Point2i & pt) const {
    cv::Point3d line_vec = _cam_model.projectPixelTo3dRay(pt);
    // get the depth - as the depth image is twice smaller than
    // the RGB one, we divide the coordinates by 2
    double dist = bridge_depth_img_ptr->image.at<float>(pt.y / 2, pt.x / 2);
    //maggiePrint("dist:%g", dist);
    return line_vec * (dist / geometry_utils::norm(line_vec));
  }

  //////////////////////////////////////////////////////////////////////////////

  static const int MAX_SAMPLE_TRIES = 20 * DEPTH_SAMPLE_SIZE;

  void remove_outliers(const cv::Mat & rgb_img, const cv::Mat & depth_img) {
    maggieDebug3("remove_outliers()");
    faces_filtered.clear();
    faces_centers_3d.clear();

    for (unsigned int face_idx = 0; face_idx < faces_not_filtered.size(); ++face_idx) {
      // get a small rect centered on the face
      cv::Rect curr_face = faces_not_filtered.at(face_idx);
      cv::Rect curr_face_small = geometry_utils::shrink_rec(curr_face, 0.5);
      // draw it
      cv::rectangle(img_out, curr_face_small, CV_RGB(255, 255, 0), 1);

      // get a sample of points located in it
      std::vector<cv::Point3d> face_sample;
      int total_samples = 0;
      while (face_sample.size() < DEPTH_SAMPLE_SIZE && total_samples < MAX_SAMPLE_TRIES) {
        ++total_samples;
        // get a 2D random point
        cv::Point curr_face_pt2d (curr_face_small.x + rand() % curr_face_small.width,
                                  curr_face_small.y + rand() % curr_face_small.height);
        // reproject it
        cv::Point3d curr_face_pt3d = reproject(curr_face_pt2d);
        // dismiss the NaN points
        if (isnan(curr_face_pt3d.x) || isnan(curr_face_pt3d.y) || isnan(curr_face_pt3d.z))
          continue;
        // keep this valid point
        face_sample.push_back(curr_face_pt3d);
        maggieDebug3("Face %i: curr_face_pt2d:'%s'\t curr_face_pt3d:'%s'",
                     face_idx,
                     geometry_utils::printP2(curr_face_pt2d).c_str(),
                     geometry_utils::printP(curr_face_pt3d).c_str());
      } // end point collecting

      // check if we could not find any points
      maggieDebug2("Face %i: We have %i sample pts, in %i tries",
                   face_idx,
                   face_sample.size(), total_samples);
      if (face_sample.size() == 0) {
        maggiePrint("Face %i: We could not find any sample points for that face !",
                    face_idx);
        continue;
      }

      // now compute the bounding box of this sample
      geometry_utils::Rect3d bbox =
          geometry_utils::boundingBox_vec3
          <double, cv::Point3d, std::vector<cv::Point3d> >(face_sample);
      //maggiePrint("Face %i: bbox:%s", face_idx, bbox.to_string().c_str());

      // tests on the geometry of the bounding box
      if (bbox.depth > MAX_SAMPLE_DEPTH)
        continue;
      if (bbox.width > MAX_SAMPLE_WIDTH)
        continue;
      // TODO

      // we passed all tests : keep the face
      faces_filtered.push_back(curr_face);
      cv::Point3d face_center_3d_cv_orien = bbox.centroid<cv::Point3d>();
      cv::Point3d face_center_3d_ros_orien;
      face_center_3d_ros_orien.x = face_center_3d_cv_orien.z;
      face_center_3d_ros_orien.y = -face_center_3d_cv_orien.x;
      face_center_3d_ros_orien.z = -face_center_3d_cv_orien.y;
      // add the radius of the head
      face_center_3d_ros_orien.x += HEAD_DEPTH;
      faces_centers_3d.push_back(face_center_3d_ros_orien);

    } // end loop face_idx
  } // end remove_outliers();

  //////////////////////////////////////////////////////////////////////////////

private:

  /**
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node, and the last
  * NodeHandle destructed will close down the node.
  */
  ros::NodeHandle n;

  message_filters::Subscriber<sensor_msgs::Image> _rgb_image_subscriber;
  message_filters::Subscriber<sensor_msgs::Image> _depth_image_subscriber;

  //! the rosmsg -> OpenCV converter
  cv_bridge::CvImageConstPtr bridge_rgb_img_ptr;
  //! the rosmsg -> OpenCV converter
  cv_bridge::CvImageConstPtr bridge_depth_img_ptr;
  //! an image for drawing stuff
  cv::Mat3b img_out;

  /* face detection */
  //! the redim image
  cv::Mat3b small_img;
  //! the list of faces found
  std::vector< cv::Rect > faces_not_filtered;
  //! the classifier
  cv::CascadeClassifier cascadeClassifier;
  //! the number of faces found
  int nb_faces;

  /* reprojection and filtering */
  sensor_msgs::CameraInfoConstPtr _cam_info;
  image_geometry::PinholeCameraModel _cam_model;
  //! the filtered list of faces found
  std::vector< cv::Rect > faces_filtered;
  //! the list of faces, with ROS orientation
  std::vector< cv::Point3d > faces_centers_3d;

  /* visu */
  cv::Mat1b depth_color_to_uchar;
  double alpha_trans, beta_trans;

  /* markers */
  ros::Publisher _marker_publisher;
  visualization_msgs::Marker _marker_msg;
}; // end class FaceDetector

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  maggieDebug2("main()");
  ros::init(argc, argv, "FaceDetector");

  FaceDetector detec;

  return 0;
}
