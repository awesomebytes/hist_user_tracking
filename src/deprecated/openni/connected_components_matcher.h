#ifndef CONNECTED_COMPONENTS_MATCHER_H
#define CONNECTED_COMPONENTS_MATCHER_H

#include <vision/utils/connected_comp/disjoint_sets2.h>
#include <vision/utils/image_utils/content_processing.h>
#include <recognition_conflict_solver.h>
#include "vision/utils/image_utils/content_processing.h"
#include "utils/geom/hausdorff_distances.h"
#include "utils/time/timer.h"

#include <deque>

//! uncomment to print recognition infos
///#define DEBUG_INFO

class ConnectedComponentsMatcher {
public:
  //! typedefs for the inputs
  typedef DisjointSets2::Comp Comp;
  typedef cv::Rect Bbox;

  //! typedef for the outputs
  typedef unsigned int FrameIdx;
  typedef int ObjectName;
  typedef DisjointSets2::CompIndex CompIdx; // also includes NO_NODE

  //! the mark of a compared component (at frame 0) with a given object at a given frame
  struct CompRecognitionMark {
    //CompIdx comp_idx;
    ObjectName possible_object_correspondance;
    FrameIdx object_frame_idx;
    float mark;
    inline std::string to_string() const {
      std::ostringstream out;
      out << /*"comp" << comp_idx <<*/ "=obj" << possible_object_correspondance
          << "@fr" << object_frame_idx << "?mark:" << mark;
      return out.str();
    }
    inline friend std::ostream & operator << (std::ostream & out,
                                              const CompRecognitionMark & mark) {
      out << mark.to_string();
      return out;
    }
  }; // end struct CompRecognitionMark


  //! a comparator function for sorting std::vectors of CompRecognitionMark
  static inline bool compare_marks(const CompRecognitionMark & m1, const CompRecognitionMark & m2) {
    return (m1.mark < m2.mark);
  }

  //! a definite succesful match of a component in a given frame with an object
  struct ObjectCompSuccesfulRecognition {
    FrameIdx comp_frame_idx;
    CompIdx comp_idx;
    //ObjectName correspondance;
  }; // end struct ObjectCompSuccesfulRecognition


  typedef std::map< ObjectName, std::vector<ObjectCompSuccesfulRecognition> > ObjectRecoHistory;

  static const int DEFAULT_WIDTH = 20, DEFAULT_HEIGHT = 20;
  static const unsigned int HISTORY_SIZE = 50;
  //! the recognition is not valid when the mark between comp and matching object gets higher
  static const float MAX_MARK_FOR_ASSOC = 0.6;
  //! the higher the factor, the more important D22 and less the bbox similarity
  static const float MARK_RATIO_BBOX_TO_D22 = 30;

  ////////////////////////////////////////////////////////////////////////////////

  //! ctor
  ConnectedComponentsMatcher() {
    resized_array = new bool [DEFAULT_WIDTH * DEFAULT_HEIGHT];
    first_available_obj_name = 0;
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! dtor
  ~ConnectedComponentsMatcher() {
    delete[] resized_array;
  }

  ////////////////////////////////////////////////////////////////////////////////


  /*! replace the last resized comps and bboxes,
      and put the other ones at the beginning of the history
    */
  inline void set_new_data(const std::vector<Comp> & components,
                           const std::vector<Bbox> & bboxes) {
    maggieDebug3("set_new_data(components.size:%i)",
                 components.size());
    // increment the frame idx of one for the recognition history
    ObjectRecoHistory::iterator obj_it = _recognition_history.begin();
    while (obj_it != _recognition_history.end()) {
      for (unsigned int obj_rec_idx = 0; obj_rec_idx < obj_it->second.size(); ++obj_rec_idx) {
        ++((obj_it->second)[obj_rec_idx].comp_frame_idx);
        maggieDebug3("name:'%i', comp%i@frame%i",
                     obj_it->first, (obj_it->second)[obj_rec_idx].comp_idx,
                     (obj_it->second)[obj_rec_idx].comp_frame_idx);
      }
      ++ obj_it;
    } // end loop obj

    // clean too old recognitions
    obj_it = _recognition_history.begin();
    while (obj_it != _recognition_history.end()) {
      std::vector<ObjectCompSuccesfulRecognition> & obj_recs = obj_it->second;
      std::vector<ObjectCompSuccesfulRecognition>::iterator rec_it = obj_recs.begin();
      // if the new frame index >= HISTORY_SIZE, throw away
      while (obj_recs.size() > 0 && rec_it != obj_recs.end()) {
        if (rec_it->comp_frame_idx >= HISTORY_SIZE) {
          obj_recs.erase(rec_it);
          rec_it = obj_recs.begin();
          continue;
        }
        ++ rec_it;
      } // end while rec_it
      ++obj_it;
    } // end loop obj_it

    // empty history -> clear object entry
    obj_it = _recognition_history.begin();
    while (obj_it != _recognition_history.end()) {
      if (obj_it->second.size() == 0) {
        _recognition_history.erase(obj_it);
        obj_it = _recognition_history.begin();
        continue;
      }
      ++ obj_it;
    } // end loop obj_it

    // store new ones
    _components_history.push_front(components);
    _bboxes_history.push_front(bboxes);
    _resized_components_history.push_front(std::vector<Comp>());
    for (CompIdx comp_idx = 0; comp_idx < (int) components.size(); ++comp_idx)
      _resized_components_history.front().push_back(Comp());

    // clean history if needed
    if (_components_history.size() > HISTORY_SIZE) {
      _components_history.pop_back();
      _resized_components_history.pop_back();
      _bboxes_history.pop_back();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*!
    \param frame_idx
      For instance, 0 = the first frame of the history
                      = the frame just before the current one.
    */
  inline void history_lookup(const FrameIdx & frame_idx,
                             const CompIdx & comp_idx,
                             const Comp* & comp_pts_ptr,
                             const Comp* & comp_resized_ptr,
                             const Bbox* & comp_bbox_ptr) const {
    maggieDebug3("history_lookup(frame_idx:%i, comp_idx:%i)",
                 frame_idx, comp_idx);
    comp_pts_ptr     = &(_components_history        [frame_idx][comp_idx]);
    comp_resized_ptr = &(_resized_components_history[frame_idx][comp_idx]);
    comp_bbox_ptr    = &(_bboxes_history            [frame_idx][comp_idx]);
    maggieDebug3("comp_pts_ptr:'%s'", geometry_utils::print_rect(*comp_bbox_ptr).c_str());
  } // end history_lookup();

  ////////////////////////////////////////////////////////////////////////////////

  /*!
    \param frame_idx
      For instance, 0 = the first frame of the history
                      = the frame just before the current one.
    \return true if found
    */
  inline bool reco_history_lookup(const ObjectName & obj_name,
                                  const FrameIdx & frame_idx,
                                  const Comp* & comp_pts_ptr,
                                  const Comp* & comp_resized_ptr,
                                  const Bbox* & comp_bbox_ptr) const {
    maggieDebug3("reco_history_lookup(obj_name:%i, frame_idx:%i)",
                 obj_name, frame_idx);
//    if (_recognition_history.count(obj_name) == 0)
//      return false;
//    const std::vector<ObjectCompSuccesfulRecognition> & reco_list =
//        _recognition_history[obj_name];
    ObjectRecoHistory::const_iterator it = _recognition_history.find(obj_name);
    if (it == _recognition_history.end())
      return false;
    //const std::vector<ObjectCompSuccesfulRecognition> & it->second = it->second;

    for (unsigned int reco_idx = 0; reco_idx < it->second.size(); ++reco_idx) {
      if (it->second[reco_idx].comp_frame_idx == frame_idx) {
        history_lookup(frame_idx, it->second[reco_idx].comp_idx,
                       comp_pts_ptr, comp_resized_ptr, comp_bbox_ptr);
        return true;
      }
    } // end loop reco_idx
    return false;
  } // end reco_history_lookup();

  ////////////////////////////////////////////////////////////////////////////////

  inline bool get_comp_name(const FrameIdx & frame_idx,
                            const CompIdx & comp_idx,
                            ObjectName & obj_name) const {
    // for all object names,
    for(ObjectRecoHistory::const_iterator obj = _recognition_history.begin();
        obj != _recognition_history.end(); ++ obj) {
      for (unsigned int obj_rec_idx = 0; obj_rec_idx < obj->second.size(); ++obj_rec_idx) {
        if (obj->second[obj_rec_idx].comp_frame_idx == frame_idx &&
            obj->second[obj_rec_idx].comp_idx == comp_idx) {
          obj_name = obj->first;
          return true;
        } // end if match
      } // end loop obj_rec_idx
    } // end loop obj
    return false;
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! return the sum of L1 distance of the two opposite corners
  static inline float bboxes_mark_L1(const cv::Rect & rect1, const cv::Rect & rect2) {
    return (hausdorff_distances::dist_L1( rect1.tl(), rect2.tl() ) +
            hausdorff_distances::dist_L1( rect1.br(), rect2.br() ) )
        / (MARK_RATIO_BBOX_TO_D22 *
           std::max(std::max(rect1.width, rect1.height),
                    std::max(rect2.width, rect2.height))
           );
  } // end bboxes_mark_L1()

  ////////////////////////////////////////////////////////////////////////////////

  inline bool comp_bboxes_mark(const CompIdx & curr_comp_idx,
                               const ObjectName & obj_name,
                               const FrameIdx & obj_frame_idx,
                               float & mark) const {
    const Comp *obj_curr_frame_pts_ptr, *obj_curr_frame_resized_ptr;
    const Bbox* obj_curr_frame_bbox_ptr = NULL;
    if (!reco_history_lookup
        (obj_name, obj_frame_idx, obj_curr_frame_pts_ptr,
         obj_curr_frame_resized_ptr, obj_curr_frame_bbox_ptr))
      return false;
    mark = bboxes_mark_L1(_bboxes_history.front()[curr_comp_idx], *obj_curr_frame_bbox_ptr);
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*! compute the D22 mark between a comp and an object at a given frame.
      Use the L1 distance instead of the euclidian L2 for faster computation.
   */
  inline bool D22_mark(const CompIdx & curr_comp_idx,
                       const ObjectName & obj_name,
                       const FrameIdx & obj_frame_idx,
                       float & mark,
                       const float max_mark = INFINITY) const {
    const Comp *obj_curr_frame_pts_ptr, *obj_curr_frame_resized_ptr = NULL;
    const Bbox* obj_curr_frame_bbox_ptr;
    if (!reco_history_lookup
        (obj_name, obj_frame_idx, obj_curr_frame_pts_ptr,
         obj_curr_frame_resized_ptr, obj_curr_frame_bbox_ptr))
      return false;
    mark = hausdorff_distances::D22_with_min(_resized_components_history.front()[curr_comp_idx],
                                   *obj_curr_frame_resized_ptr,
                                   max_mark,
                                   &hausdorff_distances::dist_L1);
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////

  void match() {
    maggieDebug3("match()");
    //Timer timer;

    // resize all components to a default size, keeping ratio
    for (unsigned int comp_idx = 0; comp_idx < _components_history.front().size(); ++comp_idx) {
      image_utils::redimContent_vector_without_repetition_given_resized_array
          (_components_history.front()[comp_idx],
           _resized_components_history.front()[comp_idx],
           cv::Rect(0, 0, DEFAULT_WIDTH - 1, DEFAULT_HEIGHT - 1),
           resized_array, true);
    } // end loop comp_idx

    //timer.printTime("after redimContent_vector_without_repetition_given_resized_array()");

    to_recompute_D22.reserve(_recognition_history.size());

    // for each comp1 of _last_components
    for (unsigned int comp1_idx = 0; comp1_idx < _components_history.front().size();
         ++comp1_idx) {
      bool was_comp1_recognized = false;
      CompRecognitionMark comp1_best_mark;
      comp1_best_mark.object_frame_idx = -1;

      // compute all the bounding boxes distances first
      // for each object object of the recognition history of the last frame
      to_recompute_D22.clear();
      ObjectRecoHistory::const_iterator obj_recs_it = _recognition_history.begin();
      for (unsigned int obj_idx = 0; obj_idx < _recognition_history.size(); ++obj_idx) {
        ObjectName obj_name = obj_recs_it->first;
        const std::vector<ObjectCompSuccesfulRecognition> & obj_recs = obj_recs_it->second;
        // we do not want to have twice the same object at different frames
        // so iterate over all recognitions and only keep the frame idx
        // with the lowest bbox mark
        float best_bbox_mark = INFINITY;
        FrameIdx best_frame_idx = 0;
        // iterate on all past recognitions of this object
        for (unsigned int obj_rec_idx = 0; obj_rec_idx < obj_recs.size(); ++obj_rec_idx) {
          if (obj_recs[obj_rec_idx].comp_frame_idx == 0)
            continue; // (warning frame 0=current frame, so skip these)
          float bbox_mark;
          bool bbox_mark_success = comp_bboxes_mark
              (comp1_idx, obj_name, obj_recs[obj_rec_idx].comp_frame_idx, bbox_mark);
          if (!bbox_mark_success)
            continue;
          if (bbox_mark < best_bbox_mark) {
            best_bbox_mark = bbox_mark;
            best_frame_idx = obj_recs[obj_rec_idx].comp_frame_idx;
          }
        } // end loop obj_rec_idx

        // now add the frame index with the lowest bbox mark
        if (best_bbox_mark < INFINITY) {
          CompRecognitionMark mark;
          //mark.comp_idx = comp1_idx;
          mark.possible_object_correspondance = obj_name;
          mark.object_frame_idx = best_frame_idx;
          mark.mark = best_bbox_mark;
          to_recompute_D22.push_back(mark);
        } // end if (best_bbox_mark < INFINITY)
        ++obj_recs_it;
      } // end loop obj_idx

      // now sort it before calling D22
      // to measure the most probables first
      std::sort(to_recompute_D22.begin(), to_recompute_D22.end(),
                compare_marks);
      maggieDebug3("to_recompute_D22(size:%i) : '%s'", to_recompute_D22.size(),
                   StringUtils::iterable_to_string(to_recompute_D22).c_str());

      // now compare with, in order, all the points of to_recompute_D22
      comp1_best_mark.object_frame_idx = -1;
      comp1_best_mark.mark = MAX_MARK_FOR_ASSOC;
      for (unsigned int to_r_idx = 0; to_r_idx < to_recompute_D22.size(); ++to_r_idx) {
        CompRecognitionMark & curr_to_rec = to_recompute_D22[to_r_idx];
        // fast skip if possible - we can skip all nexts,
        // because to_recompute_D22 is sorted by ascending marks
        if (curr_to_rec.mark > comp1_best_mark.mark)
          break;

        // otherwise, call D22
        float d22_mark = INFINITY;
        bool d22_mark_success = D22_mark
            (comp1_idx,
             curr_to_rec.possible_object_correspondance,
             curr_to_rec.object_frame_idx,
             d22_mark,
             comp1_best_mark.mark - curr_to_rec.mark);
        if (!d22_mark_success)
          continue;
#ifdef DEBUG_INFO
        maggieDebug2("Really called D22 for '%s', got d22_mark=%g",
                     curr_to_rec.to_string().c_str(), d22_mark);
#endif // DEBUG_INFO
        // keep the new mark
        curr_to_rec.mark += d22_mark;
        if (curr_to_rec.mark < comp1_best_mark.mark) {
          comp1_best_mark = curr_to_rec;
        }
      } // end loop to_r_idx

      // now see if we have succesfully recognized comp1
      if (comp1_best_mark.mark < MAX_MARK_FOR_ASSOC)
        was_comp1_recognized = true;

      // add it if recognized
      if (was_comp1_recognized) {
        ObjectCompSuccesfulRecognition comp1_reco_result;
        comp1_reco_result.comp_idx = comp1_idx;
        comp1_reco_result.comp_frame_idx = 0; // will become 1 at next increment
        _recognition_history[comp1_best_mark.possible_object_correspondance].push_back(comp1_reco_result);
#ifdef DEBUG_INFO
        maggieDebug1("comp %i was matched to object '%i' (mark:%f). ",
                     comp1_idx, comp1_best_mark.possible_object_correspondance,
                     comp1_best_mark.mark);
#endif // DEBUG_INFO
      } // end if (was_comp1_recognized)

      else {
#ifdef DEBUG_INFO
        maggieDebug1("Impossible to match comp %i to any object "
                     "(best object '%i', mark:%f). "
                     "Adding it with name '%i'.",
                     comp1_idx,
                     comp1_best_mark.object_frame_idx, comp1_best_mark.mark,
                     first_available_obj_name);
#endif // DEBUG_INFO
        ObjectCompSuccesfulRecognition comp1_reco_result;
        comp1_reco_result.comp_idx = comp1_idx;
        comp1_reco_result.comp_frame_idx = 0; // will become 1 at next increment
        // find a new available name
        _recognition_history[first_available_obj_name].push_back(comp1_reco_result);
        ++ first_available_obj_name;
      } // end if (!was_comp1_recognized)
    } // end loop comp1_idx

    //timer.printTime("end of match()");


    // conflict resolution: check if some comps have several matches and
    // take their inferior
  } // end match();

  //private:
  /*! for the histories, the most recent element
      is at the front, at index 0 */
  std::deque< std::vector<Comp> > _components_history;
  std::deque< std::vector<Comp> > _resized_components_history;
  std::deque< std::vector<Bbox> > _bboxes_history;
  ObjectRecoHistory _recognition_history;
  //! the bool buffer for redimContent_vector_without_repetition_given_resized_array()
  bool* resized_array;

  //! a name that is garanteed not to be taken by an existing object
  ObjectName first_available_obj_name;

  //! for a given component, the objects that will have to be compared with D22
  std::vector<CompRecognitionMark> to_recompute_D22;
}; // end class connected_components_matcher

#endif // CONNECTED_COMPONENTS_MATCHER_H
