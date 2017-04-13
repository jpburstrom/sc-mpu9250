sc-mpu9250

Supercollider plugin to read data from InvenSense MPU9250 over i2c.
Based on https://github.com/yourpalal/lsmd9dso-supercollider

This project will build:

 * debug: an executable that reads the data from MPU9250 over i2c and prints it
 * ahrs_debug: an executable that reads the same data, calculates roll/pitch/heading and prints that
 * MPU: a SuperCollider UGen plugin

### to build:

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ../
    make install

### to run:

    sudo ./build/apps/ahrs_debug/ahrs_debug
    sudo ./build/apps/debug/debug

### to build the debug version:

    mkdir debug
    cd debug
    cmake -DCMAKE_BUILD_TYPE=Debug ../
    make install
