#pragma once
#include "factor.h"
#include <srrg_solver/solver_core/solver_action_base.h>

namespace md_slam {
  using namespace srrg2_core;
  // holds a pool of factors
  // call setFixed, setMoving and makeFactors to populate the structure
  // afterwards add the stuff tothe graph, calling addFactors;
  struct MDFactorStack : public std::vector<MDFactorPtr> {
    void setFixed(MDImagePyramid& pyramid);
    void setMoving(MDImagePyramid& pyramid);
    // creates a factor stack, requires fixed to be set
    void makeFactors();
    // assigns fixed and moving along the pyramid to all factors
    void assignPyramids();
    void setVariableId(srrg2_solver::VariableBase::Id id);
    void addFactors(srrg2_solver::FactorGraph& graph);
    MDImagePyramid* _fixed  = 0;
    MDImagePyramid* _moving = 0;
  };

  struct MDFactorShowAction : public srrg2_solver::SolverActionBase,
                              public srrg2_core::ActiveDrawable {
    void setFactors(MDFactorStack& factors) {
      _md_factors = &factors;
    }
    void doAction() override;
    void _drawImpl(ViewerCanvasPtr gl_canvas_) const override;
    MDFactorStack* _md_factors = 0;
  };

  using MDFactorShowActionPtr = std::shared_ptr<MDFactorShowAction>;

} // namespace md_slam
