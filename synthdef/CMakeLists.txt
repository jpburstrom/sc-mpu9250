add_library(AHRS MODULE AHRS_UGen.cpp)
target_link_libraries(AHRS Debugging Adafruit_AHRS Adafruit_LSM9DS0 Adafruit_Sensor LinuxDuino)
set_target_properties(AHRS PROPERTIES PREFIX "") # not libAHRS.so

# set(SUPERCOLLIDER_INCLUDE /usr/local/include/SuperCollider)
set(SUPERCOLLIDER_INCLUDE ~/git/supercollider/include)

include_directories(AHRS ${SUPERCOLLIDER_INCLUDE}/plugin_interface ${SUPERCOLLIDER_INCLUDE}/common)


set(SUPERCOLLIDER_EXTENSION_DIR ~/.local/share/SuperCollider/Extensions/)


MESSAGE( STATUS "SUPERCOLLIDER_EXTENSION_DIR  " ${SUPERCOLLIDER_EXTENSION_DIR})
INSTALL(FILES AHRS.sc DESTINATION ${SUPERCOLLIDER_EXTENSION_DIR})
INSTALL(TARGETS AHRS DESTINATION ${SUPERCOLLIDER_EXTENSION_DIR})
