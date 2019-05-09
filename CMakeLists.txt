cmake_minimum_required(VERSION 2.8.3)
project(nclt2ros)


find_package(catkin REQUIRED COMPONENTS roscpp rospy)

catkin_python_setup()

catkin_package()

install(PROGRAMS scripts/nclt2downloader scripts/nclt2rosbag scripts/nclt2visualizer
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install rviz directory