include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("include/")
include_directories("./")
include_directories("../common/include/")
include_directories("../third-party")
include_directories("../third-party/ParamHandler")
include_directories("../third-party/vectornav/include")
include_directories("../third-party/vectornav/include/vn")

include_directories("../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("../third-party/SOEM/osal")
include_directories("../third-party/SOEM/osal/linux")
include_directories("../third-party/SOEM/oshw")
include_directories("../third-party/SOEM/oshw/linux")
include_directories("../third-party/LogitechGamepad/src")
link_libraries("motor")

file(GLOB sources "src/*.cpp"  "src/rt/*.cpp")

add_library(robot SHARED ${sources})

target_link_libraries(robot common_module pthread lcm inih dynacore_param_handler soem yesense_imu LogitechGamepad motor)
if(CMAKE_SYSTEM_NAME MATCHES Linux)
target_link_libraries(robot libvnc rt)
endif()


install(TARGETS robot   #等待安装的目标文件 可执行文件 静态库 动态库
    #   RUNTIME  DESTINATION bin                       # 安装 可执行文件到指定目录下的bin目录
      LIBRARY DESTINATION lib                        # 安装 动态库文件到指定目录下的lib目录
) 
