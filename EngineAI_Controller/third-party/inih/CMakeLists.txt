#add_library(inih STATIC ini.c ini.h INIReader.h INIReader.cpp)
add_library(inih SHARED ini.c ini.h INIReader.h INIReader.cpp)
install(TARGETS inih   #等待安装的目标文件 可执行文件 静态库 动态库
    #   RUNTIME  DESTINATION bin                       # 安装 可执行文件到指定目录下的bin目录
      LIBRARY DESTINATION lib                        # 安装 动态库文件到指定目录下的lib目录
      # ARCHIVE DESTINATION static_lib
) 