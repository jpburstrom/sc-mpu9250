NIME project using SuperCollider and LSMD9S0 from Adafruit.

This project will build:

 * debug: an executable that reads the data from LSMD9S0 over i2c and prints it
 * ahrs_debug: an executable that reads the same data, calculates roll/pitch/heading and prints that
 * AHRS: a SuperCollider UGen plugin


### to build:

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ../
    make install


### to run:

    sudo ./build/apps/ahrs_debug/ahrs_debug
    sudo ./build/apps/debug/debug
    


### to NIME


After installing, you can run  scripts/start_all.sh to start jackd and scsynth

### to build the debug version:

    mkdir debug
    cd debug
    cmake -DCMAKE_BUILD_TYPE=Debug ../
    make install
