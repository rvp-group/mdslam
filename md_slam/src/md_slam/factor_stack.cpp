#include "factor_stack.h"
#include <srrg_solver/solver_core/solver.h>

namespace md_slam {

  using namespace std;
  using namespace srrg2_core;

  // stack of factor representing pyramid
  // order -> tail, level with lower resolution (top pyramid)
  // order -> front, level with higher resolution (bottom pyramid)

  void MDFactorStack::setFixed(MDImagePyramid& pyramid) {
    _fixed = &pyramid;
  }

  void MDFactorStack::setMoving(MDImagePyramid& pyramid) {
    _moving = &pyramid;
  }

  void MDFactorStack::makeFactors() {
    // sanity check: pyramids should have same depth
    assert(_fixed && "MDFactorStack::makeFactors|not fixed set");
    size_t levels = _fixed->numLevels();
    resize(levels);
    for (size_t l = 0; l < levels; ++l) {
      MDFactorPtr& factor = at(l);
      factor.reset(new MDFactor);
      factor->setLevel(l);
    }
  }

  void MDFactorStack::assignPyramids() {
    // sanity check: pyramids should have same depth
    assert(_moving && "MDFactorStack::assignPyramids|not moving set");
    assert(_fixed && "MDFactorStack::assignPyramids|not fixed set");
    assert(_fixed->numLevels() == _moving->numLevels() &&
           "MDFactorStack::assignPyramids|fixed and moving num levels differ");

    size_t levels = _fixed->numLevels();
    for (size_t l = 0; l < levels; ++l) {
      MDPyramidLevel& l_fix = *_fixed->at(l);
      MDPyramidLevel& l_mov = *_moving->at(l);
      MDFactorPtr& factor   = at(l);
      assert(factor && "MDFactorStack::assignPyramids|not factor, forgot to call makeFactors()?");
      factor->setFixed(l_fix);
      factor->setMoving(l_mov);
    }
  }

  void MDFactorStack::setVariableId(srrg2_solver::VariableBase::Id id) {
    for (auto& f : (*this)) {
      f->setVariableId(0, id);
    }
  }

  void MDFactorStack::addFactors(srrg2_solver::FactorGraph& graph) {
    for (auto& f : (*this)) {
      graph.addFactor(f);
    }
  }

  void MDFactorShowAction::doAction() {
    if (!_md_factors) {
      return;
    }
    if (_md_factors->empty())
      return;
    _need_redraw = true;
    draw();
  }

  void MDFactorShowAction::_drawImpl(ViewerCanvasPtr gl_canvas_) const {
    if (_md_factors->empty())
      return;
    if (_solver_ptr->currentLevel() < 0 ||
        _solver_ptr->currentLevel() >= (int) _md_factors->size())
      return;
    for (auto& f : *_md_factors) {
      ImageVector3f canvas;
      f->toTiledImage(canvas);
      cv::Mat dest;
      canvas.toCv(dest);
      gl_canvas_->putImage(dest);
    }
    gl_canvas_->flush();
  }

} // namespace md_slam
