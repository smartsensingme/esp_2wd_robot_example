cmake_minimum_required(VERSION 3.16)
# This integration application deliberately uses the local component checkouts,
# including unpublished driver/LUT changes. Manifests remain portable for future
# component consumers; the app must not silently substitute managed copies.
set(ENV{IDF_COMPONENT_MANAGER} "0")
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_2wd_robot_app)
