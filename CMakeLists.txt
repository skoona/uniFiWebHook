# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(testing_ili9488)

# idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
# target_compile_options(${lvgl_lib} PRIVATE "-Wno-empty-body" "-Wno-strict-prototypes")
