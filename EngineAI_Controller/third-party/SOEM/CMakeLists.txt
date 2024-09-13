include_directories(osal)
include_directories(osal/linux)
include_directories(oshw)
include_directories(oshw/linux)
include_directories(soem)

add_library(soem SHARED
        osal/linux/osal.c
        oshw/linux/nicdrv.c
        oshw/linux/oshw.c
        soem/ethercatbase.c
        soem/ethercatcoe.c
        soem/ethercatconfig.c
        soem/ethercatdc.c
        soem/ethercatfoe.c
        soem/ethercatmain.c
        soem/ethercatprint.c
        soem/ethercatsoe.c
        )
install(TARGETS soem   #等待安装的目标文件 可执行文件 静态库 动态库
        #   RUNTIME  DESTINATION bin                       # 安装 可执行文件到指定目录下的bin目录
          LIBRARY DESTINATION lib                        # 安装 动态库文件到指定目录下的lib目录
          # ARCHIVE DESTINATION static_lib
) 