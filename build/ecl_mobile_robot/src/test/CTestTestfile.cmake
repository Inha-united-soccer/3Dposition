# CMake generated Testfile for 
# Source directory: /home/a/soccer_ws/src/turtlebot2_ros2/ecl_core/ecl_mobile_robot/src/test
# Build directory: /home/a/soccer_ws/build/ecl_mobile_robot/src/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ecl_test_partial_inverse "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/a/soccer_ws/build/ecl_mobile_robot/test_results/ecl_mobile_robot/ecl_test_partial_inverse.gtest.xml" "--package-name" "ecl_mobile_robot" "--output-file" "/home/a/soccer_ws/build/ecl_mobile_robot/ament_cmake_gtest/ecl_test_partial_inverse.txt" "--command" "/home/a/soccer_ws/build/ecl_mobile_robot/src/test/ecl_test_partial_inverse" "--gtest_output=xml:/home/a/soccer_ws/build/ecl_mobile_robot/test_results/ecl_mobile_robot/ecl_test_partial_inverse.gtest.xml")
set_tests_properties(ecl_test_partial_inverse PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/a/soccer_ws/build/ecl_mobile_robot/src/test/ecl_test_partial_inverse" TIMEOUT "60" WORKING_DIRECTORY "/home/a/soccer_ws/build/ecl_mobile_robot/src/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/a/soccer_ws/src/turtlebot2_ros2/ecl_core/ecl_mobile_robot/src/test/CMakeLists.txt;6;ament_add_gtest;/home/a/soccer_ws/src/turtlebot2_ros2/ecl_core/ecl_mobile_robot/src/test/CMakeLists.txt;22;ecl_add_mobile_robot_gtest;/home/a/soccer_ws/src/turtlebot2_ros2/ecl_core/ecl_mobile_robot/src/test/CMakeLists.txt;0;")
subdirs("../../gtest")
