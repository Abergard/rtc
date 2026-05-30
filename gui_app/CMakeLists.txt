cmake_minimum_required(VERSION 3.10)

project(rtc_gui_app CXX)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(OpenGL REQUIRED)

add_executable(rtc_gui_app
  source/image_conversion.cpp
  source/main.cpp
  source/main_window.cpp
  source/scene_view.cpp
)

set_target_properties(rtc_gui_app PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
  AUTOMOC OFF
)

target_link_libraries(rtc_gui_app
  PRIVATE
    input_model
    rtc_image_generation
    Qt5::Widgets
    OpenGL::GL
)

file(COPY ${CMAKE_SOURCE_DIR}/assets/brs_xml/ DESTINATION ${CMAKE_BINARY_DIR}/gui_app/assets/brs_xml/)
