cmake_minimum_required(VERSION 2.8.3)
project(wheel)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# Build the executable
add_executable(wheel
        ./src/wheel.cpp
        lib/src/MySerial.cpp
        lib/src/net_serial.cpp
        lib/src/timer.cpp
        )
# Add a build order dependency on nav_msgs
# This ensures that nav_msgs' msg headers are built before your executable
if(nav_msgs_EXPORTED_TARGETS)
    add_dependencies(wheel ${nav_msgs_EXPORTED_TARGETS})
endif()

target_link_libraries(wheel
  ${catkin_LIBRARIES} 
)
