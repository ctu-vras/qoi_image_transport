# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(qoi_image_transport)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  cras_topic_tools
  image_transport
  image_transport_codecs
  pluginlib
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qoi qoi_codec
  CATKIN_DEPENDS cras_cpp_common cras_topic_tools image_transport_codecs sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(qoi src/qoi.c
  src/qoi_codec_plugin.cpp)

add_library(qoi_codec src/qoi_codec.cpp)
target_link_libraries(qoi_codec PUBLIC ${catkin_LIBRARIES} PRIVATE qoi)
add_dependencies(qoi_codec ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(qoi_codec_plugin src/qoi_codec_plugin.cpp)
target_link_libraries(qoi_codec_plugin ${catkin_LIBRARIES} qoi_codec)
add_dependencies(qoi_codec_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME} src/qoi_publisher.cpp src/qoi_subscriber.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} qoi_codec)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS qoi qoi_codec qoi_codec_plugin ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES
  codec.xml
  transport.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Roslint C++ - checks formatting and some other rules for C++ files
  file(GLOB_RECURSE ROSLINT_FILES include/*.h src/*.c src/*.cpp test/*.cpp)
  list(FILTER ROSLINT_FILES EXCLUDE REGEX ".*/qoi.[ch]")
  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-build/include,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_FILES})

  # Roslint C++ - checks formatting and some other rules for C++ files
  file(GLOB_RECURSE ROSLINT_PY_FILES src/*.py test/*.py)
  roslint_python(${ROSLINT_PY_FILES})

  roslint_add_test()

  ## Add gtest based cpp test target and link libraries
  # catkin_add_gtest(${PROJECT_NAME}-test test/test_qoi_image_transport.cpp)
  # if(TARGET ${PROJECT_NAME}-test)
  #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  # endif()

  ## Add folders to be run by python nosetests
  # catkin_add_nosetests(test)
endif()
