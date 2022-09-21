cmake_minimum_required(VERSION 3.0.2)
project(haptics_controller_choreonoid_sample)

find_package(catkin REQUIRED COMPONENTS
  jsk_models
  hrpsys_choreonoid
  hrpsys_choreonoid_tutorials)

catkin_package()

configure_file(${PROJECT_SOURCE_DIR}/config/TABLIS_BASE_RH_FLAT.cnoid.in ${PROJECT_SOURCE_DIR}/config/TABLIS_BASE_RH_FLAT.cnoid @ONLY)
