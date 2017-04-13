#include <pthread.h>
#include <atomic>
#include "SC_Lock.h"
#include "SC_PlugIn.h"

#include "MPU9250.h"
#include "debug.h"


// written with reference to the chapter "Writing Unit Generator Plug-ins" in The SuperCollider Book
// and also http://doc.sccode.org/Guides/WritingUGens.html accessed March 2, 2015


struct MPU : public Unit {
    //Outputs 3 values
    static const int ACCEL = 0;
    static const int GYRO = 1;
    static const int MAG = 2;
    static const int ORIENTATION = 3;
    //Outputs single values
    static const int PITCH  = 4;
    static const int ROLL = 5;
    static const int YAW = 6;

    int channel = ACCEL;
    int outputs = 3;
};

mpu9250State_t gData = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

// PLUGIN INTERFACE

extern "C" {
    void MPU_Ctor(MPU *unit);
    //void MPU_next_k1(MPU *unit, int numSamples);
    void MPU_next_k(MPU *unit, int numSamples);
}


static InterfaceTable *ft;

// PLUGIN IMPLEMENTATION
MPU9250 mpu;

void MPU_Ctor(MPU *unit) {
    unit->channel = static_cast<int>(IN0(0));
    unit->outputs = (unit->channel > MPU::ORIENTATION) ? 1 : 3;

    SETCALC(MPU_next_k);
    MPU_next_k(unit, 1);
}

void MPU_next_k(MPU *unit, int numSamples) {

    float value[3];
    int outputs = unit->outputs;

    switch (unit->channel) {
        case MPU::GYRO:
            value[0] = gData.gx;
            value[1] = gData.gy;
            value[2] = gData.gz;
            break;
        case MPU::MAG:
            value[0] = gData.mx;
            value[1] = gData.my;
            value[2] = gData.mz;
            break;
        case MPU::ORIENTATION:
            value[0] = gData.pitch;
            value[1] = gData.roll;
            value[2] = gData.yaw;
            break;
        case MPU::PITCH:
            value[0] = gData.pitch;
            break;
        case MPU::ROLL:
            value[0] = gData.roll;
            break;
        case MPU::YAW:
            value[0] = gData.yaw;
            break;
        case MPU::ACCEL:
        default:
            value[0] = gData.ax;
            value[1] = gData.ay;
            value[2] = gData.az;
            break;
    }

    for (int i = 0; i < numSamples ; i++) {
        for (int o = 0; o < outputs; o++) {
            OUT(o)[i] = value[o];
            //OUT(0) = 0;
            //OUT(1) = 0;
            //OUT(2) = 0;
        }
    }
        //    printf("roll:%f", gData.roll);
}

//Threading stuff
std::atomic_bool inputThreadRunning = { false };

void *gstate_update_func(void *param) {
    if ( mpu.init() ) {
        while ( inputThreadRunning.load( std::memory_order_relaxed ) ) {
            mpu.read(gData);
            std::this_thread::sleep_for( std::chrono::milliseconds( 15 ) );
        }
    }
    return NULL;
}

pthread_t mpuThread;

PluginLoad(MPU)
{
    debug("PluginLoad")

    ft = inTable;
    inputThreadRunning = true;
    //TODO: do we need to start thread here?
    pthread_create(&mpuThread, NULL, gstate_update_func, &gData);

    //mpuThread = std::thread(gstate_update_func);

    DefineSimpleUnit(MPU);
}

C_LINKAGE SC_API_EXPORT void unload(InterfaceTable *inTable)
{
	inputThreadRunning = false;
    pthread_join(mpuThread, NULL);
}
