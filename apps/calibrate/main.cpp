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
        printf("WARN NG: Some percentages seem to be a bit off.\n");
    } else {
        printf("Seems ok.\n");
    }

    usleep(1e06);

    if (!mpu.init()) {
        std::cerr <<  "sensor init failed!" << std::endl;
        return 1;
    } else {
        cout << " nit ok\n";
    }

    if (!mpu.testRW()) {
        std::cerr <<  "Can't write to i2c device!" << std::endl;
        return 1;
    } else {
        cout << "R/W OK\n";
    }

    FILE * pFile;
    pFile = fopen("calibration.bin", "rb");
    if (pFile != NULL) {
        mpu9250Calibration_t data;
        fread(&data, sizeof(mpu9250Calibration_t), 1, pFile);
        mpu.setCalibration(data);
        fclose(pFile);
        printf("Testing: Accel x bias = %f", mpu.calibration.aBias[0]);
    }


    printf("Calibrate Gyro/Accel? (y/N)");
    scanf(" %c", &userInput);

    if (userInput == 'y') {
        //Calibrate accel + gyro
        cout << "Calibrating Accel + Gyro\n";
        mpu.calibrateAccelGyro();
        printf("Accel bias: %f, %f, %f\n", mpu.calibration.aBias[0], mpu.calibration.aBias[1], mpu.calibration.aBias[2]) ;
        printf("Gyro bias: %f, %f, %f\n", mpu.calibration.gBias[0], mpu.calibration.gBias[1], mpu.calibration.gBias[2]) ;

        mpu.init(); // reset various stuff
    }


    printf("Calibrate magnetometer? (y/N)");
    scanf(" %c", &userInput);


    if (userInput == 'y') {

        cout << "Calibrating Magnetometer\n";
        cout << "Wave device in a figure eight until done!\n";
        mpu.calibrateMag();
        printf("Mag bias: %f, %f, %f\n", mpu.calibration.mBias[0], mpu.calibration.mBias[1], mpu.calibration.mBias[2]) ;
        printf("Mag scale: %f, %f, %f\n", mpu.calibration.mScale[0], mpu.calibration.mScale[1], mpu.calibration.mScale[2]) ;

    }

    printf("Save calibration? (y/N)");
    scanf(" %c", &userInput);

    if (userInput == 'y') {
        pFile = fopen("calibration.bin", "wb");
        if (pFile != NULL) {
            mpu9250Calibration_t data;
            mpu.getCalibration(data);
            fwrite(&data, sizeof(mpu9250Calibration_t), 1, pFile);
            fclose(pFile);
        }

    }


    int count = 0;

    while (running) {
        mpu.read();

        if (count == 0) {
            printf("\rAX: %.02f AY: %.02f AZ: %.02f\t P: %.02f R: %.02f Y: %.02f \t GX: %.02f GY: %.02f GZ: %.02f\t MX: %.02f MY: %.02f MZ: %.02f", 
                    mpu.accel.x, mpu.accel.y, mpu.accel.z, mpu.orientation.pitch, mpu.orientation.roll, mpu.orientation.yaw,
                    mpu.gyro.x, mpu.gyro.y, mpu.gyro.z, mpu.mag.x, mpu.mag.y, mpu.mag.z);

        }
        count = (count + 1) % 10;
        fflush(stdout);
        usleep(2500); //400Hz
    }
}
