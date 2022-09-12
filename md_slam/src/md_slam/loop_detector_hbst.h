#pragma once
#include <srrg_hbst/types/binary_matchable.hpp>
#include <srrg_hbst/types/binary_tree.hpp>

#include "loop_detector_base.h"

namespace md_slam_closures {

  class LoopDetectorHBST : public md_slam_closures::LoopDetectorBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief a bit of usings
    using ThisType = LoopDetectorHBST;
    using BaseType = LoopDetectorBase;

    static constexpr size_t DESCRIPTOR_SIZE_BITS = BaseType::DESCRIPTOR_SIZE_ORB;
    using BinaryEntry       = srrg_hbst::BinaryMatchable<FramePoint*, DESCRIPTOR_SIZE_BITS>;
    using BinaryEntryNode   = srrg_hbst::BinaryNode<BinaryEntry>;
    using BinaryEntryVector = srrg_hbst::BinaryNode<BinaryEntry>::MatchableVector;
    using HSBTTree          = srrg_hbst::BinaryTree<BinaryEntryNode>;

    PARAM(srrg2_core::PropertyUnsignedInt,
          max_descriptor_distance,
          "maximum distance (Hamming) between descriptors",
          25,
          nullptr);
    PARAM(srrg2_core::PropertyFloat,
          minimum_matching_ratio,
          "minimum ratio between the number of framepoints in the query and in the match",
          0.1f,
          nullptr);
    PARAM(srrg2_core::PropertyBool,
          enable_matches_filtering,
          "enables matches container filtering based on the match score",
          true,
          nullptr);

    LoopDetectorHBST();
    virtual ~LoopDetectorHBST() = default;

    //! @brief resets the tree
    void reset() override;

    //! @brief adds the current frame to the tree and performs also a query
    void compute() override;

  protected:
    //! @brief hbst tree
    HSBTTree _tree;
  };

  using LoopDetectorHBSTPtr = std::shared_ptr<LoopDetectorHBST>;
} // namespace md_slam_closures
