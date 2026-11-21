cmake_minimum_required(VERSION 3.5)
project(atdog_description VERSION 3.0.0)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

foreach(dir meshes urdf mjcf)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
        install(
            DIRECTORY ${dir}/
            DESTINATION share/${PROJECT_NAME}/${dir}
        )
    endif()
endforeach()

# Preserve relative includes used by scene_tag.xml in the installed tree.
install(DIRECTORY ../common/ DESTINATION share/common)

ament_package()
