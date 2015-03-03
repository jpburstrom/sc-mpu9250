set(SUPERCOLLIDER_INCLUDE ~/git/supercollider/include)

add_library(AHRS MODULE AHRS_UGen.cpp)
target_link_libraries(AHRS Debugging Adafruit_AHRS Adafruit_LSM9DS0 Adafruit_Sensor LinuxDuino)
set_target_properties(AHRS PROPERTIES PREFIX "") # not libAHRS.so
include_directories(AHRS ${SUPERCOLLIDER_INCLUDE}/plugin_interface ${SUPERCOLLIDER_INCLUDE}/common)


find_library(wiringPi NAMES wiringPi PATHS /usr/local/lib/)

add_library(GPIO MODULE GPIO_UGen.cpp)
target_link_libraries(GPIO Debugging wiringPi)
set_target_properties(GPIO PROPERTIES PREFIX "") # not libGPIO.so

include_directories(GPIO ${SUPERCOLLIDER_INCLUDE}/plugin_interface ${SUPERCOLLIDER_INCLUDE}/common /usr/local/include/)


set(SUPERCOLLIDER_EXTENSION_DIR ~/.local/share/SuperCollider/Extensions/)


MESSAGE( STATUS "SUPERCOLLIDER_EXTENSION_DIR  " ${SUPERCOLLIDER_EXTENSION_DIR})
INSTALL(FILES AHRS.sc GPIO.sc DESTINATION ${SUPERCOLLIDER_EXTENSION_DIR})
INSTALL(TARGETS AHRS GPIO DESTINATION ${SUPERCOLLIDER_EXTENSION_DIR})
