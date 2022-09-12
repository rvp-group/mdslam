#include <bits/stdc++.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_pcl/point_types.h>

#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

#include <srrg_image/image.h>

#include <iostream>
#include <vector>

#include "md_slam/factor.h"
#include "md_slam/pyramid_variable_se3.h"
#include "md_slam/utils.h"
#include <srrg_solver/solver_core/iteration_algorithm_gn.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>

#include <unistd.h>

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace md_slam;

using PointVector = PointNormalIntensity3fVectorCloud;

const char* banner[] = {"extracts a timestamped trajectory from a md-slam graph", 0};

int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString graph_name(&cmd_line, "i", "input", "graph name to load", "");
  ArgumentString out_name(&cmd_line, "o", "output", "graph name to load", "");
  cmd_line.parse();

  if (!graph_name.isSet()) {
    std::cerr << std::string(environ[0]) + "|ERROR, no input provided, aborting" << std::endl;
    return -1;
  }
  FactorGraphPtr graph = FactorGraph::read(graph_name.value());
  if (!graph) {
    std::cerr << std::string(environ[0]) + "|ERROR, unable to load graph" << std::endl;
    return -1;
  }
  std::cerr << "graph loaded, n vars: " << graph->variables().size()
            << " factors:" << graph->factors().size() << endl;

  std::ostream* os = &cout;
  ofstream out;
  if (!out_name.isSet()) {
    std::cerr << std::string(environ[0]) + "|WARNING, no output provided, dumping to stdout"
              << std::endl;
  } else {
    out.open(out_name.value());
    os = &out;
  }
  *os << "#timestamp tx ty tz qx qy qz qw" << std::endl;
  for (size_t i = 1; i < graph->variables().size(); ++i) {
    VariableBase* v_base = graph->variable(i);
    MDVariableSE3* v     = dynamic_cast<MDVariableSE3*>(v_base);
    if (!v)
      continue;
    *os << fixed << std::setprecision(10) << v->timestamp() << " ";
    *os << fixed << std::setprecision(6);
    // need to flip quaternion from [qw, qx, qy, qz] to [qx, qy, qz, qw]
    const auto trans = geometry3d::t2tnqw(v->estimate()).head(3);
    const auto quat  = geometry3d::t2tnqw(v->estimate()).tail(4);
    *os << trans.transpose() << " " << quat(1) << " " << quat(2) << " " << quat(3) << " " << quat(0)
        << endl;
  }
}
