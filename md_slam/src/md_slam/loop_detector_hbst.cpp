#include "loop_detector_hbst.h"

namespace md_slam_closures {

  LoopDetectorHBST::LoopDetectorHBST() {
  }

  void LoopDetectorHBST::reset() {
  }

  void LoopDetectorHBST::compute() {
    // sanity check and containers' reset
    BaseType::compute();

    // get the framepoints
    const size_t& num_framepoints = BaseType::_query_frame->numFramePoints();

    // create the matching entries
    BinaryEntryVector entries;
    entries.reserve(num_framepoints);
    for (FramePoint* f : *BaseType::_query_frame->framepoints()) {
      entries.emplace_back(new BinaryEntry(f, f->descriptor, BaseType::_query_frame->id()));
    }

    assert(entries.size() == BaseType::_query_frame->numFramePoints() &&
           "LoopDetectorHBST::compute|invalid framepoint number");

    // query the tree and populate it at the same time
    HSBTTree::MatchVectorMap matches;
    _tree.matchAndAdd(entries, matches, param_max_descriptor_distance.value());

    // obviously if the frame distance is not enough just fuck you
    const size_t& frame_interspace = param_frame_interspace.value();
    if (((int) BaseType::_query_frame->id() - (int) frame_interspace) < 0) {
      // std::cerr << "LoopDetectorHBST::compute|WARNING, skipping due to frame
      // interspace\n";
      return;
    }

    // ia discriminate the matches, computing the best match
    // ia HSBTTree::MatchVectorMap is a vector in which each
    // ia element is the match_vector for every scene added til now.
    size_t best_index = 0;
    float best_ratio  = 0.f;

    // ia cache parameters
    const bool enable_matches_filtering = param_enable_matches_filtering.value();
    const float minimum_matching_ratio  = param_minimum_matching_ratio.value();

    // ia cache all returned matches (unfiltered)
    BaseType::_matched_frame_vector.reserve(matches.size());
    for (const auto& match_id_match_vector_pair : matches) {
      const size_t& image_number_reference = match_id_match_vector_pair.first;

      // check that the reference respects the frame interspace
      if ((BaseType::_query_frame->id() >= frame_interspace &&
           image_number_reference <= BaseType::_query_frame->id() - frame_interspace) ||
          frame_interspace == 0) {
        // cache
        const auto& match_vector = match_id_match_vector_pair.second;
        // TODO check that this fucking match has some stuff inside (WTF!)
        if (match_vector.empty()) {
          continue;
        }

        assert(match_vector.begin()->object_references.size() > 0 &&
               "LoopDetectorHBST::compute|invalid match object references");

        // compute current match's score
        float current_ratio = (float) match_vector.size() / num_framepoints;
        if (current_ratio > best_ratio) {
          best_ratio = current_ratio;
          best_index = image_number_reference;
        }

        // collect the reference frame associated to this hbst match
        if (enable_matches_filtering) { // we discard most association based
                                        // on min threshold
          if (current_ratio >= minimum_matching_ratio) {
            Frame* database_frame = match_vector[0].object_references[0]->parent_frame;
            assert(database_frame && "LoopDetectorHBST::compute|ERROR, corrupted framepoint");
            BaseType::_matched_frame_vector.emplace_back(database_frame);
            BaseType::_scores.insert(std::make_pair(database_frame->id(), current_ratio));
          }
        } else { // we take all association, takes few days per frame
          Frame* database_frame = match_vector[0].object_references[0]->parent_frame;
          assert(database_frame && "LoopDetectorHBST::compute|corrupted framepoint");
          BaseType::_matched_frame_vector.emplace_back(database_frame);
          BaseType::_scores.insert(std::make_pair(database_frame->id(), current_ratio));
        }
      }
    }

    // bdc from now on, we have in matches[best_index].second the correspondences,
    // i.e. the scene entries constraints
    const size_t num_framepoint_matched = matches[best_index].size();
    // std::cerr << "LoopDetectorHBST::compute|candidate matches [ "
    //           << BaseType::_matched_frame_vector.size() << " ]\n";
    // std::cerr << "LoopDetectorHBST::compute|best-match ID [ " << best_index
    //           << " ] -- matching ratio [ " << best_ratio << " ] aka [ " <<
    //           num_framepoint_matched
    //           << " / " << num_framepoints << " ] framepoints\n";

    // take the current frame and generate associations
    if (best_ratio >= minimum_matching_ratio) {
      BaseType::_associations.reserve(num_framepoint_matched);
      BaseType::_matched_frame = matches[best_index][0].object_references[0]->parent_frame;
      BaseType::_best_score    = best_ratio;
      for (const auto& hbst_matchable : matches[best_index]) {
        BaseType::_associations.emplace_back(
          FramePointAssociation(hbst_matchable.object_query, hbst_matchable.object_references[0]));
      }
    }
  }

} // namespace md_slam_closures
