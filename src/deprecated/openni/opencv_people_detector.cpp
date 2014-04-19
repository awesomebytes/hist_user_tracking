// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <cv_bridge/cv_bridge.h>
// AD
#include <utils/debug/debug.h>
#include <utils/time/timer.h>
#include <vision/utils/image_utils/drawing_utils.h>

// cf http://experienceopencv.blogspot.com/2011/02/hog-descriptor.html

class OpenCvPeopleDetector {
public:

  //! the scale reduction factor of the RGB image for face detection
  static const double SCALE = 0.4;

  //////////////////////////////////////////////////////////////////////////////

  OpenCvPeopleDetector() {
    // subscribe to the image message
    _depth_image_subscriber = n.subscribe<sensor_msgs::Image>
        ("/camera/rgb/image_color",
         1,
         &OpenCvPeopleDetector::image_callback,
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
    if(!img_ptr.data)
      return;

    // prepare the small image
    cv::resize(img_ptr, small_img, cv::Size(0, 0), SCALE, SCALE, cv::INTER_NEAREST);

    cv::HOGDescriptor hog;
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    //hog.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());

#if 1
    // run the detector with default parameters. to get a higher hit-rate
    // (and more false alarms, respectively), decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    std::vector<cv::Rect> found_rectangles;
    hog.detectMultiScale(small_img, // const Mat& img,
                         found_rectangles,  // CV_OUT vector<Rect>& foundLocations,
                         0, // double hitThreshold=0
                         cv::Size(8,8), // Size winStride=Size()
                         cv::Size(32,32), // Size padding=Size()
                         1.05, // double scale=1.05
                         2 // double finalThreshold=2.0
                         );

    // filter the rectangles
    std::vector<cv::Rect> found_rectangles_filtered;
    size_t i, j;
    for( i = 0; i < found_rectangles.size(); i++ ) {
      cv::Rect r = found_rectangles[i];
      for( j = 0; j < found_rectangles.size(); j++ )
        if( j != i && (r & found_rectangles[j]) == r)
          break;
      if( j == found_rectangles.size() )
        found_rectangles_filtered.push_back(r);
    }

    // rescale found faces
    for ( std::vector< cv::Rect >::iterator r = found_rectangles_filtered.begin();
          r != found_rectangles_filtered.end() ; ++r) {
      r->x /= SCALE;
      r->y /= SCALE;
      r->width /= SCALE;
      r->height /= SCALE;
    } // end loop found_rectangles

    // draw
    img_ptr.copyTo(img_out);
    for( i = 0; i < found_rectangles_filtered.size(); i++ ) {
      cv::Rect r = found_rectangles_filtered[i];
      // the HOG detector returns slightly larger rectangles than the real objects.
      // so we slightly shrink the rectangles to get a nicer output.
      r.x += cvRound(r.width*0.1);
      r.width = cvRound(r.width*0.8);
      r.y += cvRound(r.height*0.07);
      r.height = cvRound(r.height*0.8);
      rectangle(img_out, r.tl(), r.br(), cv::Scalar(0,255,0), 3);
    }

#else

    std::vector<cv::Point> found_pts;
    hog.detect(img_ptr, // const Mat& img,
               found_pts, // CV_OUT vector<Point>& foundLocations,
               0, // double hitThreshold=0
               cv::Size(8,8), // Size winStride=Size()
               cv::Size(32,32) // Size padding=Size()
               //const vector<Point>& searchLocations=vector<Point>()
               );

    // draw
    img_ptr.copyTo(img_out);
    for (unsigned int pt_idx = 0; pt_idx < found_pts.size(); ++pt_idx)
      cv::circle(img_out, found_pts[pt_idx], 4, CV_RGB(0, 0, 255), 2);
#endif // 0


    /*
     * now build the marker message
     */
    // ...
    // emit it
    //_marker_publisher.publish(_marker_msg);

    timer.printTime("image_callback();");

    cv::imshow("img_out", img_out);
    int key_code = (char) cv::waitKey(3);
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

  //! the rosmsg -> OpenCV converter
  cv_bridge::CvImageConstPtr bridge_img_ptr;
  //! the redim image
  cv::Mat1b small_img;
  //! an image for drawing stuff
  cv::Mat3b img_out;

  ros::Subscriber _depth_image_subscriber;
  ros::Publisher _marker_publisher;

  visualization_msgs::Marker _marker_msg;
}; // end class OpenCvPeopleDetector

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  maggieDebug2("main()");
  ros::init(argc, argv, "OpenCvPeopleDetector");

  OpenCvPeopleDetector detec;

  return 0;
}

