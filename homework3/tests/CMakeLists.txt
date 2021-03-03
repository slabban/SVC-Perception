find_package(rostest REQUIRED)

add_rostest_gtest(test_homework3
  homework3.test
  homework3_test.cpp
)
add_dependencies(test_homework3
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_homework3
  ${catkin_LIBRARIES}
)

catkin_download_test_data(
  homework3.bag
  http://www.secs.oakland.edu/~mtradovn/ece_6460/homework3.bag
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
  FILENAME homework3.bag
  MD5 bb90d17993d5cfe758b41c36d639ae57
)