cmake_minimum_required(VERSION 3.15)
project(meat-wagons)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
link_libraries(ws2_32 wsock32)

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.h")
add_executable(meat-wagons ${SRC})
