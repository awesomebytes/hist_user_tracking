// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/image_encodings.h>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
// AD
#include <utils/debug/debug.h>
#include <utils/time/timer.h>

class ImageSubscriberTemplate {
public:

  //////////////////////////////////////////////////////////////////////////////

  ImageSubscriberTemplate() {
    // subscribe to the image message
    _depth_image_subscriber = n.subscribe<sensor_msgs::Image>
        ("/camera/rgb/image_color",
         1,
         &ImageSubscriberTemplate::image_callback,
         this);

    _marker_publisher = n.advertise<visualization_msgs::Marker>("cluster_markers", 10);

    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
    ros::spin();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! this function is called each time an image is received
  void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    maggieDebug2("image_callback()");

    Timer timer;
    // conversion to cv::Mat, cf
    // http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    try {
      bridge_img_ptr = cv_bridge::toCvShare(msg,
                                           sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    const cv::Mat & img_ptr = bridge_img_ptr->image;

    // do the stuff

    /*
     * now build the marker message
     */
    // ...
    // emit it
    //_marker_publisher.publish(_marker_msg);

    timer.printTime("image_callback()");

    cv::imshow("img_ptr", img_ptr);
    int key_code = (char) cv::waitKey(1);
    if (key_code == 27)
      exit(-1);

  } // end image_callback();

  //////////////////////////////////////////////////////////////////////////////

private:

  /**
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node, and the last
  * NodeHandle destructed will close down the node.
  */
  ros::NodeHandle n;

  ros::Subscriber _depth_image_subscriber;

  //! the rosmsg -> OpenCV converter
  cv_bridge::CvImageConstPtr bridge_img_ptr;

  ros::Publisher _marker_publisher;
  visualization_msgs::Marker _marker_msg;
}; // end class ImageSubscriberTemplate

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  maggieDebug2("main()");
  ros::init(argc, argv, "ImageSubscriberTemplate");

  ImageSubscriberTemplate detec;

  return 0;
}

