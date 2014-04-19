#ifndef FAST_CLUSTER_FUNCTIONS_H
#define FAST_CLUSTER_FUNCTIONS_H

#include <opencv2/core/core.hpp>
#include "vision/utils/image_utils/drawing_utils.h"
#include "vision/utils/image_utils/color_utils.h"

namespace fast_cluster_functions {

static const int NO_OBJECT = -1;

//! the topic where we emit the GUI image
static const std::string GUI_TOPIC = "/fast_cluster_detector_img_gui";

//! the topic where we emit the image containing the objects name
static const std::string OBJECTS_IMAGE_TOPIC = "/fast_cluster_detector_objects_img";

//! the topic where we emit the number of objects in the detector
static const std::string OBJECTS_NB_TOPIC = "/fast_cluster_detector_objects_nb";

//! the topic where we emit the name of the tracked object
static const std::string TRACKED_OBJECT_NAME_TOPIC = "/fast_cluster_detector_tracked_object_name";

//! the topic where we emit the 3D position of the tracked object
static const std::string TRACKED_OBJECT_POSE_TOPIC = "/moving_goal";

/*!
 \param objects_names_img
    -1 for NO OBJECT
 \param components_illus
*/
inline void paint_object_image(const cv::Mat1i & objects_names_img,
                               cv::Mat3b & components_illus) {
  components_illus.create(objects_names_img.size());
  components_illus = cv::Vec3b(0, 0, 0);
  const int* objects_names_img_ptr = &(objects_names_img(0));
  cv::Vec3b* components_illus_ptr = &(components_illus(0));

  //  for (int row = 0; row < objects_names_img.rows; ++row) {
  //    // get the address of row
  //    const int* objects_names_img_ptr = objects_names_img.ptr<int>(row);
  //    cv::Vec3b* components_illus_ptr = components_illus.ptr<cv::Vec3b>(row);
  //    for (int col = 0; col < objects_names_img.cols; ++col) {
  int n_values = objects_names_img.cols * objects_names_img.rows;
  for (int value_idx = 0; value_idx < n_values; ++value_idx) {
    if (*objects_names_img_ptr != NO_OBJECT)
      color_utils::indexed_color255((*components_illus_ptr)[0],
                                    (*components_illus_ptr)[1],
                                    (*components_illus_ptr)[2],
                                    *objects_names_img_ptr);
    ++objects_names_img_ptr;
    ++components_illus_ptr;
  } // end loop value_idx
  //} // end loop col
  //} // end loop row

} // end transform_object_();

} // end namesapce fast_cluster_functions


#endif // FAST_CLUSTER_FUNCTIONS_H
