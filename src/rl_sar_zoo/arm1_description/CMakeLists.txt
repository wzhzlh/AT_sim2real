cmake_minimum_required(VERSION 3.5)
project(arm1_description VERSION 1.0.0)

find_package(ament_cmake REQUIRED)

foreach(dir config launch meshes mjcf urdf)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
        install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
    endif()
endforeach()

ament_package()
