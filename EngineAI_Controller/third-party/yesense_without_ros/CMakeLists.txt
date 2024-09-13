cmake_minimum_required(VERSION 2.8.3)
project(yesense_imu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   serial
# )

find_package(Boost COMPONENTS system filesystem REQUIRED)

message(STATUS ${PROJECT_SOURCE_DIR})

add_subdirectory(third_party/serial)

add_library(yesense_imu SHARED
            "src/analysis_data.cpp"
            "src/yesense_driver.cpp")
            
target_link_libraries(yesense_imu serial boost_thread ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
target_include_directories(yesense_imu PUBLIC 
./src
./third_party/serial/include)

## Declare a C++ executable
add_executable(test_yesense_imu 
  # src/yesense_driver.cpp
  src/yesense_node.cpp
  # src/analysis_data.cpp
)

target_link_libraries(test_yesense_imu yesense_imu)

# add debug info
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O0 -Wall")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -g")

#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS yesense_imu test_yesense_imu
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# install (DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install (DIRECTORY rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
