# Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

cmake_minimum_required(VERSION 3.12)
project(robcom-667)

find_package(YARP 3.8.1 REQUIRED COMPONENTS os sig math)
find_package(ICUB 2.4.0 REQUIRED COMPONENTS iKin)
find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ICUB::iKin)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

icubcontrib_add_uninstall_target()
