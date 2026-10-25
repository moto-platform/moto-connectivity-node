# ESP-IDF project file, used by PlatformIO for `framework = arduino, espidf`.
cmake_minimum_required(VERSION 3.16.0)
# Application version (D-036): ESP-IDF puts PROJECT_VER into the image's app description
# (esp_app_get_description()->version). Bump it in the release PR.
set(PROJECT_VER "0.4.0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(moto-connectivity-node)
