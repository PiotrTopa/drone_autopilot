cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ttgo)
spiffs_create_partition_image(storage font FLASH_IN_PROJECT)