# CMake generated Testfile for 
# Source directory: /home/a/soccer_ws/src/turtlebot2_ros2/ecl_core/ecl_mpl/src/test
# Build directory: /home/a/soccer_ws/build/ecl_mpl/src/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_enable_if "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/a/soccer_ws/build/ecl_mpl/test_results/ecl_mpl/test_enable_if.gtest.xml" "--package-name" "ecl_mpl" "--output-file" "/home/a/soccer_ws/build/ecl_mpl/ament_cmake_gtest/test_enable_if.txt" "--command" "/home/a/soccer_ws/build/ecl_mpl/src/test/test_enable_if" "--gtest_output=xml:/home/a/soccer_ws/build/ecl_mpl/test_results/ecl_mpl/test_enable_if.gtest.xml")
set_tests_properties(test_enable_if PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/a/soccer_ws/build/ecl_mpl/src/test/test_enable_if" TIMEOUT "60" WORKING_DIRECTORY "/home/a/soccer_ws/build/ecl_mpl/src/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/a/soccer_ws/src/turtlebot2_ros2/ecl_core/ecl_mpl/src/test/CMakeLists.txt;5;ament_add_gtest;/home/a/soccer_ws/src/turtlebot2_ros2/ecl_core/ecl_mpl/src/test/CMakeLists.txt;0;")
subdirs("../../gtest")
