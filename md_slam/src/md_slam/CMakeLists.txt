add_library(md_slam_library SHARED
  # basic stuff
  utils.cpp   
  image_pyramid.cpp   
  pyramid_level.cpp 
  pyramid_message.cpp 
  pyramid_generator.cpp 
  pyramid_variable_se3.cpp   
  # tracker stuff
  factor_common.cpp 
  factor.cpp 
  factor_stack.cpp 
  pairwise_aligner.cpp
  tracker_status_message.cpp 
  tracker.cpp 
  graph_manager.cpp 
  tracker_viewer.cpp
  # closure stuff 
  frame.cpp
  loop_closer.cpp
  loop_detector_base.cpp
  loop_detector_hbst.cpp
  loop_validator.cpp
  instances.cpp 
)

target_link_libraries(md_slam_library
  srrg2_solver_core_library
  srrg2_point_cloud_library
  srrg2_messages_library
  srrg2_data_structures_library
  srrg2_solver_types3d_library
  srrg2_image_library
  ${OpenCV_LIBS}
)
