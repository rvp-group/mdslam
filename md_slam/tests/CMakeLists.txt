catkin_add_gtest(test_factor_depth_camera
  test_factor_depth_camera.cpp
  )

target_link_libraries(test_factor_depth_camera
  ${catkin_LIBRARIES}
  srrg2_data_structures_library
  srrg2_point_cloud_library
  srrg2_image_library
  md_slam_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  pthread
)


catkin_add_gtest(test_factor_lidar
  test_factor_lidar.cpp
  )

target_link_libraries(test_factor_lidar
  ${catkin_LIBRARIES}
  srrg2_data_structures_library
  srrg2_point_cloud_library
  srrg2_image_library
  md_slam_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  pthread
)