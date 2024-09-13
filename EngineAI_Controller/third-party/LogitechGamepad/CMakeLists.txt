cmake_minimum_required(VERSION 3.5)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

project(LogitechGamepad)

add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-fPIC>")

# Get C or C++ sources in the current directory (only).
set(CMAKE_COLOR_MAKEFILE  ON)

# include_directories(src/Logitech_controller.h)

add_library(LogitechGamepad SHARED src/Logitech_controller.cpp)

# Setup the target executable.
add_executable(LogitechGamepad_test Logitech_controller_test.cpp)
target_link_libraries(LogitechGamepad_test LogitechGamepad)


## Mark executables and/or libraries for installation
install(TARGETS LogitechGamepad LogitechGamepad_test
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

