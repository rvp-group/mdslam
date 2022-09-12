#define GL_GLEXT_PROTOTYPES 1
#include <bits/stdc++.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_pcl/point_types.h>

#include <srrg_image/image.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

#include <iostream>
#include <vector>

#include "md_slam/utils.h"
#include "viewer/md_slam_viewer.h"
#include <qapplication.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>

#include <unistd.h>

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace md_slam;

const char* banner[] = {"loads a graph with pyramids attached and displays everyhting", 0};

int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString graph_name(&cmd_line, "i", "input", "graph name to load", "");
  cmd_line.parse();
  if (!graph_name.isSet()) {
    std::cerr << std::string(environ[0]) + "|ERROR, no input provided, aborting" << std::endl;
    return -1;
  }
  FactorGraphPtr graph = FactorGraph::read(graph_name.value());
  if (!graph) {
    std::cerr << std::string(environ[0]) + "|ERROR, unable to load graph, aborting" << std::endl;
    return -1;
  }
  std::cerr << std::string(environ[0]) + "|ERROR, graph loaded, n vars: "
            << graph->variables().size() << " factors:" << graph->factors().size() << std::endl;

  QApplication app(argc, argv);
  // instantiate the viewer
  std::mutex proc_mutex;
  MDViewerPtr viewer(new MDViewer(graph, proc_mutex));
  viewer->setWindowTitle("graph");
  // make the viewer window visible on screen
  viewer->show();
  return app.exec();
}
