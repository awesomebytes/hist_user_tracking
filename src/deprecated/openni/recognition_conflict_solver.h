#ifndef RECOGNITION_CONFLICT_SOLVER_H
#define RECOGNITION_CONFLICT_SOLVER_H

#include <vector>
#include <list>

namespace recognition_conflict_solver {

//! the object storing the possible correspondencies
template<class _UnknownObjectType, class _RecognizedObjectType>
class RecognitionMark {
  _UnknownObjectType obj;
  _RecognizedObjectType possible_reco;
  double mark;
}; // end class RecognitionMark

template<class _UnknownObjectType, class _RecognizedObjectType>
std::list<std::pair<_UnknownObjectType, _RecognizedObjectType> >
solve_conflicts
(std::list<RecognitionMark<_UnknownObjectType, _RecognizedObjectType> >) {

} // end solve_conflicts()

} // end namespace recognition_conflict_solver

#endif // RECOGNITION_CONFLICT_SOLVER_H
