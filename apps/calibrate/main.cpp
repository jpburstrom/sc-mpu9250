#include <MPU9250.h>
#include <signal.h>
#include <unistd.h>
#include <iostream>

bool running = true;

void kill_handler(int s) {
    running = false;
}

int main(int argc, char** argv) {

    // setup ctrl-c handler
    // code from http://stackoverflow.com/questions/1641182/how-can-i-catch-a-ctrl-c-event-c
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = kill_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);


    MPU9250 mpu;

    float selfTest[6];

    char userInput;

    mpu.MPU9250SelfTest(selfTest); // Start by performing self test and reporting values
    printf("x-axis self test: acceleration trim within %f%% of factory value\n", selfTest[0]);
    printf("y-axis self test: acceleration trim within %f%% of factory value\n", selfTest[1]);
    printf("z-axis self test: acceleration trim within %f%% of factory value\n", selfTest[2]);
    printf("x-axis self test: gyration trim within %f%% of factory value\n", selfTest[3]);
    printf("y-axis self test: gyration trim within %f%% of factory value\n", selfTest[4]);
    printf("z-axis self test: gyration trim within %f%% of factory value\n", selfTest[5]);
    
    bool warn=false;
    for (int i=0; i < 6; i++) {
        if (fabs(selfTest[i]) > 14) {
            warn = true;
        }
    }

    if (warn) {
        printf("WARNING: Some percentages seem to be a bit off.\n");
    } else {
        printf("Seems ok.\n");
    }

    usleep(1e06);

    if (!mpu.init()) {
        std::cerr <<  "sensor init failed!" << std::endl;
        return 1;
    } else {
        cout << "Init ok\n";
    }

    if (!mpu.testRW()) {
        std::cerr <<  "Can't write to i2c device!" << std::endl;
        return 1;
    } else {
        cout << "R/W OK\n";
    }



    //Calibrate accel + gyro
    cout << "Calibrating Accel + Gyro\n";
    mpu.calibrateAccelGyro();
    printf("Accel bias: %f, %f, %f\n", mpu.accel.bias[0], mpu.accel.bias[1], mpu.accel.bias[2]) ;
    printf("Gyro bias: %f, %f, %f\n", mpu.gyro.bias[0], mpu.gyro.bias[1], mpu.gyro.bias[2]) ;

    mpu.init(); // reset various stuff


    printf("Calibrate magnetometer? (y/n)");
    scanf("%c", &userInput);

    if (userInput == 'y') {
   
        cout << "Calibrating Magnetometer\n";
        cout << "Wave device in a figure eight until done!\n";
        mpu.calibrateMag();
        printf("Mag bias: %f, %f, %f\n", mpu.mag.bias[0], mpu.mag.bias[1], mpu.mag.bias[2]) ;
        printf("Mag scale: %f, %f, %f\n", mpu.mag.scale[0], mpu.mag.scale[1], mpu.mag.scale[2]) ;
    
    }

    while (running) {
        mpu.read();
        printf("\rAccel X: %.02f\t Accel Y: %.02f\t Accel Z: %.02f\t Pitch: %.02f\t Roll: %.02f\t Yaw: %.02f", 
                mpu.accel.x, mpu.accel.y, mpu.accel.z, mpu.orientation.pitch, mpu.orientation.roll, mpu.orientation.yaw);

        fflush(stdout);
        usleep(10000);
    }
}
