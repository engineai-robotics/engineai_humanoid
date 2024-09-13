cmake_minimum_required(VERSION 3.5)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

if($ENV{WEBOTS_HOME})
  message("home WEBOTS_HOME.found")
else()
  message("home WEBOTS_HOME not found.")
endif()
# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})
set(CMAKE_COLOR_MAKEFILE   ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(BUILD_TYPE_RELEASE TRUE)
option(NO_SIM "Do not build simulator" OFF)
option(USE_LOG_DEBUG_HL "open std::cout" OFF)
option(USE_JN_SDU_BIPED "change imu quat and gyro acc data" OFF)
if(USE_JN_SDU_BIPED)
  add_definitions(-DUSE_JN_SDU_BIPED)
endif()
if(USE_LOG_DEBUG_HL)
  add_definitions(-DUSE_LOG_DEBUG_HL)
endif()
SET (THIS_COM "../" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)

  set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=core2 -mtune=core2 -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wshadow -Wsign-promo -Wcpp")
  
  # -mno-default 
  set(CMAKE_C_FLAGS "-O3 -march=native  -ggdb -std=gnu99 -I.")
  message("**** engineAI robot build enabled ****")
set(CMAKE_CXX_STANDARD 14)

include_directories("./")
add_subdirectory(robot)
add_subdirectory(third-party)
add_subdirectory(common)
add_subdirectory(user)
include_directories(SYSTEM "third-party/qpOASES/include")
# Link with the Webots controller library.

#link_directories( $ENV{WEBOTS_HOME}/lib/controller)
link_directories("/usr/local/webots/lib/controller")
set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories(SYSTEM "/usr/local/webots/include/controller/c" "/usr/local/webots/include/controller/cpp")

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})

target_link_libraries(${PROJECT} engineai_humanoid_controller robot common_module qpOASES osqp Goldfarb_Optimizer pthread lcm inih dynacore_param_handler soem)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/build/EngineAI_Humanoid/install)
install(TARGETS ${PROJECT}   
      RUNTIME  DESTINATION bin                       
) 

# install config files
install(FILES config/zqsa01-robot-default-param.yaml
        DESTINATION ${CMAKE_INSTALL_PREFIX}/config
        )

install(PROGRAMS scripts/run_biped.sh
        DESTINATION ${CMAKE_INSTALL_PREFIX}/../
        )
