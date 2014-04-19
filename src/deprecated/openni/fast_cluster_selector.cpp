#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// AD
#include <utils/debug/debug.h>
#include <utils/time/timer.h>
// hist_user_tracking
#include "fast_cluster_functions.h"

ros::Publisher object_publisher;
cv::Mat3b components_illus;
const cv::Mat1i* objects_img_ptr;
const std::string WINDOW_NAME = fast_cluster_functions::OBJECTS_IMAGE_TOPIC;

////////////////////////////////////////////////////////////////////////////////

void mouse_callback(int event, int x, int y, int, void*) {
  // only keep left button clicks
  if (event != CV_EVENT_LBUTTONDOWN)
    return;
  cv::Point comp_pos(x, y);
  int obj_name = (*objects_img_ptr)(comp_pos);

  if (obj_name == fast_cluster_functions::NO_OBJECT) {
    maggiePrint("The user clicked on a non object point. Stopping to track.");
  } else {
    maggiePrint("Starting to track object %i.", obj_name);
  }
  std_msgs::Int32 msg; msg.data = obj_name;
  object_publisher.publish(msg);
}

////////////////////////////////////////////////////////////////////////////////

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
  //maggieDebug2("image_callback()");
  //Timer timer;
  cv_bridge::CvImageConstPtr bridge_img_ptr;
  try {
    bridge_img_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  objects_img_ptr = (const cv::Mat1i*) &(bridge_img_ptr->image);
  fast_cluster_functions::paint_object_image(*objects_img_ptr, components_illus);
  //timer.printTime("paint_object_image()");

  cv::imshow(WINDOW_NAME, components_illus);
  cv::waitKey(50);
  //timer.printTime("imshow()");
} // end image_callback();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  maggieDebug2("main()");
  ros::init(argc, argv, "fast_cluster_selector");
  // subscribe to image topic
  ros::NodeHandle nh;
  image_transport::ImageTransport transport(nh);
  image_transport::Subscriber sub = transport.subscribe
      (fast_cluster_functions::OBJECTS_IMAGE_TOPIC, 1, image_callback);
  // create publisher
  object_publisher = nh.advertise<std_msgs::Int32>(fast_cluster_functions::TRACKED_OBJECT_NAME_TOPIC, 1);
  // create window
  cv::namedWindow(WINDOW_NAME);
  cv::setMouseCallback(WINDOW_NAME, mouse_callback);
  // spin!
  ros::spin();
  return 0;
}


