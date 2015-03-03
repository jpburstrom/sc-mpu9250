cmake_minimum_required(VERSION 2.8)

project (Nime)

set(CMAKE_POSITION_INDEPENDENT_CODE True)
message(${CMAKE_CXX_FLAGS_RELEASE})

# hack to get c++11 + warnings
add_definitions(-std=c++11 -Wall)

include_directories(libs/LinuxDuino libs/Adafruit_Sensor libs/Adafruit_LSM9DS0_Library libs/Adafruit_AHRS libs/debugging)

add_subdirectory(libs/LinuxDuino)
add_subdirectory(libs/Adafruit_Sensor)
add_subdirectory(libs/Adafruit_LSM9DS0_Library)
add_subdirectory(libs/Adafruit_AHRS)
add_subdirectory(libs/debugging)

add_subdirectory(apps/debug)
add_subdirectory(apps/ahrs_debug)

add_subdirectory(synthdef)
