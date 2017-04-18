cmake_minimum_required(VERSION 2.8)

set(CMAKE_POSITION_INDEPENDENT_CODE True)
message(${CMAKE_CXX_FLAGS_RELEASE})

# hack to get c++11 + warnings
add_definitions(-std=c++11 -Wall)

include_directories(libs/mpu9250 libs/debugging)

add_subdirectory(libs/mpu9250)
add_subdirectory(libs/debugging)

add_subdirectory(apps/debug)
add_subdirectory(apps/calibrate)
add_subdirectory(apps/ahrs_debug)

add_subdirectory(plugin)
