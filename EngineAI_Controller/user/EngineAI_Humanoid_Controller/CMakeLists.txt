include_directories("./")
include_directories("../../robot/include/")
include_directories("../../common/include/")
include_directories("./Controllers/")
include_directories("../../third-party/")
include_directories(SYSTEM "../../third-party/qpOASES/include")
include_directories("../../third-party/ParamHandler")
include_directories("../../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
link_libraries("motor")

file(GLOB_RECURSE sources
        "./*.cpp"
        "FSM_States/*.cpp")

add_subdirectory(Controllers/RL_Controller)
add_library(engineai_humanoid_controller SHARED ${sources} EngineAI_Humanoid_Controller.cpp)

find_package(yaml-cpp REQUIRED)

target_link_libraries(engineai_humanoid_controller 
robot
common_module
qpOASES
osqp
Goldfarb_Optimizer
pthread
lcm
inih
dynacore_param_handler 
soem 
rl_controllers
yaml-cpp
motor
)

install(TARGETS engineai_humanoid_controller
      LIBRARY DESTINATION lib
) 
