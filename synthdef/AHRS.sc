AHRS : UGen {
   *kr {
     arg channel = 0, mul = 1.0 add = 0;

     ^this.multiNew('control', channel).madd(mul, add)
   }

    *ROLL = 0;
    *PITCH = 1;
    *HEADING = 2;
}
