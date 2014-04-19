#include "utils/debug/debug.h"
#include "connected_components_matcher.h"
#include "vision/utils/image_utils/drawing_utils.h"
#include "vision/utils/image_utils/color_utils.h"

inline void test_given_patterns(ConnectedComponentsMatcher & matcher,
                                cv::Mat1b & patterns,
                                DisjointSets2 & set,
                                bool display_matches = false,
                                bool time_display_ms = 0) {
  // get the connected comps
  set.process_image(patterns);
  std::vector<ConnectedComponentsMatcher::Comp> comps;
  std::vector<ConnectedComponentsMatcher::Bbox> bboxes;
  set.get_connected_components(patterns.cols, comps, bboxes);

  Timer timer;
  matcher.set_new_data(comps, bboxes);
  timer.printTime("after set_new_data()");
  matcher.match();
  timer.printTime("after match()");

#if 0 // display resized components
  for (unsigned int comp_idx = 0; comp_idx < matcher._last_resized_components.size(); ++comp_idx) {
    patterns = 0;
    image_utils::drawListOfPoints
        (patterns, matcher._last_resized_components[comp_idx], (uchar) 180);
    cv::imshow("patterns", patterns); cv::waitKey(0);
  } // end loop comp_idx
#endif

  // display matches
  if (display_matches) {
    cv::Mat3b illus(patterns.size());
    for (unsigned int comp_idx = 0; comp_idx < comps.size(); ++comp_idx) {
      ConnectedComponentsMatcher::ObjectName obj_name = 0;
      bool success = matcher.get_comp_name(0, comp_idx, obj_name);
      if (!success) {
        maggiePrint("Impossible to retrieve the name of %i", comp_idx);
        continue;
      }
      image_utils::drawListOfPoints
          (illus, comps[comp_idx], color_utils::color<cv::Vec3b>(obj_name));
      cv::putText(illus, StringUtils::cast_to_string(obj_name), bboxes[comp_idx].tl(),
                  CV_FONT_HERSHEY_PLAIN, 2, color_utils::color_scalar<cv::Scalar>(obj_name));
    } // end loop comp_idx
    cv::imshow("illus", illus);
    char c = cv::waitKey(time_display_ms);
    if (c == (char) 27)
      exit(0);
  } // end if display_matches
}

////////////////////////////////////////////////////////////////////////////////

inline void test_moving_numbers(bool display) {
  ConnectedComponentsMatcher matcher;

  // generate history
  unsigned int n_comp = 11;
  float max_speed = 3;
  cv::Mat1b patterns(480, 640);
  DisjointSets2 set;
  std::vector<cv::Point> centers;
  std::vector<cv::Point2f> directions;
  for (unsigned int center = 0; center < n_comp; ++center) {
    centers.push_back(cv::Point
                      (rand() % (patterns.cols / 2) + patterns.cols / 4,
                       rand() % (patterns.rows / 2) + patterns.rows / 4));
    double angle = drand48() * 2 * PI;
    directions.push_back(cv::Point2f(max_speed * cos(angle),
                                     max_speed * sin(angle)));
  }

  while (true) {
    patterns = 0;
    for (unsigned int comp_idx = 0; comp_idx < n_comp - 1; ++comp_idx) {
      centers[comp_idx].x  += directions[comp_idx].x;
      centers[comp_idx].y  += directions[comp_idx].y;
      // prevent from going out
      if (centers[comp_idx].x < 50)
        directions[comp_idx].x = std::abs(directions[comp_idx].x);
      else if (centers[comp_idx].x > patterns.cols - 50)
        directions[comp_idx].x = -std::abs(directions[comp_idx].x);
      if (centers[comp_idx].y < 50)
        directions[comp_idx].y = std::abs(directions[comp_idx].y);
      else if (centers[comp_idx].y > patterns.rows - 50)
        directions[comp_idx].y = -std::abs(directions[comp_idx].y);
      // generate the img
      cv::putText(patterns,
                  StringUtils::cast_to_string<char>('A' + comp_idx),
                  centers[comp_idx],
                  CV_FONT_HERSHEY_PLAIN, 3 + drand48() / 2, cv::Scalar::all(255),
                  3);
    } // end loop comp_idx
    //cv::imshow("patterns", patterns); cv::waitKey(10);
    test_given_patterns(matcher, patterns, set, display, 50);
  } // end loop t
}

////////////////////////////////////////////////////////////////////////////////

inline void test_increasing_numbers(bool display) {
  ConnectedComponentsMatcher matcher;

  // generate history
  unsigned int n_comp = 8;
  cv::Mat1b patterns(480, 640);
  DisjointSets2 set;
  for (float t = 0; t < 30; t+=0.5) {
    patterns = 0;
    for (unsigned int comp_idx = 0; comp_idx < n_comp - 1; ++comp_idx) {
      // generate the img
      std::string text = StringUtils::cast_to_string<int>(comp_idx);
      cv::putText(patterns,
                  text,
                  cv::Point(40 + 40 * comp_idx, 100 + 20 * cos(t)),
                  CV_FONT_HERSHEY_PLAIN, 4 + 2 * cos(t), cv::Scalar::all(255),
                  3);
    } // end loop comp_idx
    //cv::imshow("patterns", patterns); cv::waitKey(10);
    test_given_patterns(matcher, patterns, set, display, 0);
  } // end loop t
}

////////////////////////////////////////////////////////////////////////////////

int main(int, char**) {
  maggieDebug2("main()");
  srand(time(NULL));
  maggiePrint("1: test_increasing_numbers()");
  maggiePrint("2: test_moving_numbers()");
  int choice = 1;
  //std::cin >> choice;
  if (choice == 1)
    test_increasing_numbers(false);
  else
    test_moving_numbers(true);
  return 0;
}

