#ifndef FACE_RECOGNIZER_H
#define FACE_RECOGNIZER_H

#include <opencv2/highgui/highgui.hpp>

class SingleFaceClassifier {
public:
  typedef cv::Mat1f ParamMatrix;
  typedef cv::Mat1f Face;

  static const int FACE_WIDTH = 30;
  static const int FACE_HEIGHT = 40;

  /*! the cost function for a given face,
      once the ParamMatrix has been trained
    */
  static double cost_function(const ParamMatrix & theta,
                              const Face & face);

  /*! the training function */
  void train(const std::vector<Face> & faces);

private:
  cv::Mat1f _theta;
}; // end class SingleFaceClassifier

////////////////////////////////////////////////////////////////////////////////

/*! \class  FaceRecognizer
 *
 */
class FaceRecognizer {
public:
  /*! constructor */
  FaceRecognizer();

  void load_existing_faces();

  void compute_regression(const cv::Mat1b & face);
private:

}; // end class FaceRecognizer

#endif // FACE_RECOGNIZER_H
