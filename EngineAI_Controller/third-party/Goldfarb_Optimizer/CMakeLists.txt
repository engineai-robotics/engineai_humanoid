FILE(GLOB_RECURSE headers *.hh)
FILE(GLOB_RECURSE sources *.cc)

set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb ") # no werror here!


add_library (Goldfarb_Optimizer SHARED ${headers} ${sources})
#add_library (Goldfarb_Optimizer STATIC ${headers} ${sources})

#install(TARGETS Goldfarb_Optimizer DESTINATION "${INSTALL_LIB_DIR}")
#install(FILES ${headers} DESTINATION
    #"${INSTALL_INCLUDE_DIR}/Optimizer/Goldfarb_Optimizer")
install(TARGETS Goldfarb_Optimizer   #等待安装的目标文件 可执行文件 静态库 动态库
    #   RUNTIME  DESTINATION bin                       # 安装 可执行文件到指定目录下的bin目录
      LIBRARY DESTINATION lib                        # 安装 动态库文件到指定目录下的lib目录
      # ARCHIVE DESTINATION static_lib
) 