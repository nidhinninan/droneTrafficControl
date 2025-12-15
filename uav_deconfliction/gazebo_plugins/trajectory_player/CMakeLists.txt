cmake_minimum_required(VERSION 3.10)
project(trajectory_player_plugin)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(trajectory_player_plugin SHARED trajectory_player_plugin.cpp)
target_link_libraries(trajectory_player_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)

install(TARGETS trajectory_player_plugin DESTINATION lib)
