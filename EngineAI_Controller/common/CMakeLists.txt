# Source/Include files
include_directories(".")
include_directories("include/" )                 # common includes
include_directories("/usr/local/include/lcm/")   # lcm includes

include_directories("../third-party/inih")       # inih includes
include_directories("../third-party/osqp/include")       # osqp includes
include_directories("../third-party/ParamHandler")
include_directories("/usr/include/eigen3")   # lcm includes
include_directories("/usr/local/webots/lib/controller")
include_directories(SYSTEM "/usr/local/webots/include/controller/c" "/usr/local/webots/include/controller/cpp")
link_libraries("motor")

file(GLOB_RECURSE sources "src/*.cpp")                   # common library cpp files

set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

# Library
add_library(common_module SHARED ${sources})       # produce a library used by sim/robot
target_link_libraries(common_module inih dynacore_param_handler JCQP osqp motor)

install(TARGETS common_module
      LIBRARY DESTINATION lib
) 

