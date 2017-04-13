MPU : MultiOutUGen {
    init { arg ... args; 
        inputs = args;
        /*
        Channels:
        0: Accel
        1: Gyro
        2: Mag
        3: Orientation (pitch, roll, yaw)
        4: Pitch
        5: Roll
        6: Yaw
        */
        if (args[0] < 4) {
            //Output 3 values
            ^this.initOutputs(3, rate);
        } {
            ^this.initOutputs(1, rate);
        }
    }
    *kr {
        arg channel = 0, mul = 1.0, add = 0;

        ^this.multiNew('control', channel).madd(mul, add)
    }

    *accelKr {
        arg mul = 1.0, add = 0;
        ^this.kr(0, mul, add);
    }

    *gyroKr {
        arg mul = 1.0, add = 0;
        ^this.kr(1, mul, add);
    }

    *magKr {
        arg mul = 1.0, add = 0;
        ^this.kr(2, mul, add);
    }

    *orientationKr {
        arg mul = 1.0, add = 0;
        ^this.kr(3, mul, add);
    }

    *pitchKr{
        arg mul = 1.0, add = 0;
        ^this.kr(4, mul, add);
    }

    *rollKr {
        arg mul = 1.0, add = 0;
        ^this.kr(5, mul, add);
    }

    *yawKr{
        arg mul = 1.0, add = 0;
        ^this.kr(6, mul, add);
    }
}
