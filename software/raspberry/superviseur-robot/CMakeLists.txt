cmake_minimum_required(VERSION 3.28)
project(superviseur_robot)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(lib)
include_directories(lib/base64)

add_executable(superviseur_robot
        lib/base64/base64.cpp
        lib/base64/base64.h
        lib/base64/test.cpp
        lib/camera.cpp
        lib/camera.h
        lib/commonitor.cpp
        lib/commonitor.h
        lib/comrobot.cpp
        lib/comrobot.h
        lib/img.cpp
        lib/img.h
        lib/messages.cpp
        lib/messages.h
        nbproject/private/c_standard_headers_indexer.c
        nbproject/private/cpp_standard_headers_indexer.cpp
        main.cpp
        tasks.cpp
        tasks.h
        wrapper.c)
