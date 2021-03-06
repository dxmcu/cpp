cmake_minimum_required(VERSION 3.5)
project(speech_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-fpic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(Qt5Core_DIR /opt/Qt5.12.3/5.12.3/gcc_64/lib/cmake/Qt5Core)
find_package(Qt5Core CONFIG REQUIRED)
set(Qt5Gui_DIR /opt/Qt5.12.3/5.12.3/gcc_64/lib/cmake/Qt5Gui)
find_package(Qt5Gui CONFIG REQUIRED)
set(Qt5Widgets_DIR /opt/Qt5.12.3/5.12.3/gcc_64/lib/cmake/Qt5Widgets)
find_package(Qt5Widgets CONFIG REQUIRED)
set(Qt5Multimedia_DIR /opt/Qt5.12.3/5.12.3/gcc_64/lib/cmake/Qt5Multimedia)
find_package(Qt5Multimedia CONFIG REQUIRED)

FILE(GLOB SRC_FILES "src/*.cpp")
FILE(GLOB HEAD_FILES "include/*.h")
FILE(GLOB UI_FILES "src/*.ui")
add_executable(speech_test ${SRC_FILES} ${HEAD_FILES} ${RCC_FILES} ${UI_FILES} common/messagetranslate.hpp)

ament_target_dependencies(speech_test rclcpp std_msgs)

target_link_libraries(speech_test Qt5::Widgets Qt5::Multimedia jsoncpp)

target_include_directories(speech_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS speech_test
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
