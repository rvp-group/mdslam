#include "loop_detector_base.h"

namespace md_slam_closures {

  LoopDetectorBase::LoopDetectorBase() {
  }

  void LoopDetectorBase::compute() {
    assert(_query_frame && "LoopDetectorBase::compute|ERROR, no query");
    _matched_frame = nullptr;
    _matched_frame_vector.clear();
    _best_score = 0;
    _scores.clear();
    _associations.clear();
  }

} // namespace md_slam_closures
