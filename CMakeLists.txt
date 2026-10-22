cmake_minimum_required(VERSION 3.16)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" PACKAGE_XML_CONTENT)
string(REGEX MATCH "<version>([^<]+)</version>" _PACKAGE_VERSION_MATCH "${PACKAGE_XML_CONTENT}")
if(NOT _PACKAGE_VERSION_MATCH)
  message(FATAL_ERROR "Could not determine project version from package.xml")
endif()

project(unbag VERSION "${CMAKE_MATCH_1}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Svg)

ament_python_install_package(ros2_unbag)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)

Python3_add_library(_native MODULE src/native/lzf.cpp)
set_target_properties(_native PROPERTIES AUTOMOC OFF AUTORCC OFF AUTOUIC OFF)
# Optimize the kernel even when colcon does not specify a build type.
target_compile_options(_native PRIVATE
  "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<CXX_COMPILER_ID:GNU,Clang>>:-O3>"
)
install(TARGETS _native LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/ros2_unbag")

set(UNBAG_EGG_INFO_DIR "${CMAKE_CURRENT_BINARY_DIR}/python_metadata/unbag.egg-info")
set(ROS2_UNBAG_INSTALL_EGG_INFO_DIR
  "${PYTHON_INSTALL_DIR}/unbag-${PROJECT_VERSION}-py${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}.egg-info"
)
add_custom_command(
  OUTPUT "${UNBAG_EGG_INFO_DIR}/PKG-INFO"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/python_metadata"
  COMMAND ${Python3_EXECUTABLE} setup.py egg_info --egg-base "${CMAKE_CURRENT_BINARY_DIR}/python_metadata"
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  DEPENDS
    setup.py
    package.xml
    ros2_unbag/export.py
  COMMENT "Generating unbag egg-info metadata"
)
add_custom_target(unbag_python_metadata ALL
  DEPENDS "${UNBAG_EGG_INFO_DIR}/PKG-INFO"
)

add_executable(ros2_unbag_gui
  src/gui/main.cpp
  src/gui/bridge_client.cpp
  src/gui/main_window.cpp
  src/gui/processor_chain_widget.cpp
  qt_resources/resources.qrc
)

target_include_directories(ros2_unbag_gui PRIVATE src/gui)
target_link_libraries(ros2_unbag_gui PRIVATE Qt5::Core Qt5::Widgets Qt5::Svg)

install(TARGETS ros2_unbag_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  "${UNBAG_EGG_INFO_DIR}/"
  DESTINATION "${ROS2_UNBAG_INSTALL_EGG_INFO_DIR}"
)

install(FILES
  resource/unbag
  DESTINATION share/ament_index/resource_index/packages
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  file(GLOB ROS2_UNBAG_PYTESTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "tests/test_*.py")
  foreach(pytest_file IN LISTS ROS2_UNBAG_PYTESTS)
    get_filename_component(pytest_name "${pytest_file}" NAME_WE)
    ament_add_pytest_test("${pytest_name}" "${pytest_file}")
  endforeach()
endif()

ament_package()
