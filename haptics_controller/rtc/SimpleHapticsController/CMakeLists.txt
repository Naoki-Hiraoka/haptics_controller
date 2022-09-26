add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  cpp_filters
  joint_limit_table
  )
find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  )
link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_library(SimpleHapticsController SHARED
  SimpleHapticsController.cpp
  SimpleHapticsControllerService_impl.cpp
  )
target_link_libraries(SimpleHapticsController
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
  )
set_target_properties(SimpleHapticsController PROPERTIES PREFIX "")
add_dependencies(SimpleHapticsController RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild2_genidl

add_executable(SimpleHapticsControllerComp SimpleHapticsControllerComp.cpp)
target_link_libraries(SimpleHapticsControllerComp SimpleHapticsController)

install(TARGETS SimpleHapticsController
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
