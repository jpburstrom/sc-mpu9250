/*
This software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt
*/


#include <cmath>
#include <signal.h>

#include <MPU9250.h>

//TEST
#include <OSCClient.h>
OSCClient oscClient;

// Define this to print data to terminal

volatile sig_atomic_t flag = 0;

int readCount = 0;			// How long until we read again...
int readIntervalSamples = 0; // How many samples between reads
int readInterval = 10;

void readMPU9250();

MPU9250 mpu;

void cleanup(int sig)
{
	// Nothing to do here
    flag = 1;
}


// Auxiliary task to read the I2C board
void readMPU9250()
{
	mpu.getData();
	oscClient.sendMessageNow(
		oscClient.newMessage.to("/c_setn").add(64).add(11)
		.add(mpu.ax).add(mpu.ay).add(mpu.az)
		.add(mpu.gx).add(mpu.gy).add(mpu.gz)
		.add(mpu.mx).add(mpu.my).add(mpu.mz)
		.add(mpu.pitch).add(mpu.roll).end());
}



int main()
{

    signal(SIGINT, cleanup);

	oscClient.setup(57110);
	
	//Initialize MPU
	if(!mpu.begin()) {
		printf("Error initialising MPU9250\n");
		return 1;
	}

	//MPU selftest 
	//mpu.MPU9250SelfTest(mpu.selfTest);
	//Accel XYZ, Gyration XYZ
	//printf("A: xyz %f %f %f\n", mpu.selfTest[0], mpu.selfTest[1], mpu.selfTest[2]);
	//printf("G: xyz %f %f %f\n", mpu.selfTest[3], mpu.selfTest[4], mpu.selfTest[5]);


    // Calibrate gyro and accelerometers, load biases in bias registers
    //XXX floating point exception
    //Can't read from FIFO
    mpu.calibrateMPU9250();
#ifdef DEBUG
    printf("Gyro bias %f %f %f\n", mpu.gyroBias[0], mpu.gyroBias[1], mpu.gyroBias[2]);
    printf("Accel bias %f %f %f\n", mpu.accelBias[0], mpu.accelBias[1], mpu.accelBias[2]);
#endif
   	
    mpu.initMPU9250();
    
    printf("init ready");
    
    delay(1000);
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    
    int16_t dest[3];
    mpu.readAccelData(dest);
#ifdef DEBUG
    printf("Accel %i %i %i\n", dest[0], dest[1], dest[2]);
#endif

    mpu.readGyroData(dest);
    printf("Gyro %i %i %i\n", dest[0], dest[1], dest[2]);

   
    
    mpu.initAK8963();
    //mpu.magcalMPU9250();
    
    
	//mpu.calibrateMPU9250(mpu.gyroBias, mpu.accelBias);
    
    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
   	uint8_t d = mpu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    printf("I should be 0x48: %#04x\n", d);
    
    if (d != 0x48) {
    	return false;
    }
    
	mpu.readMagData(dest);
	printf("Mag %i %i %i\n", dest[0], dest[1], dest[2]);
	
	
    while (true) {
        if (flag) {
            mpu.closeI2C();
            exit(0);
        } else {
            readMPU9250();
            delay(readInterval);
        }
    }

}

