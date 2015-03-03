AHRS : UGen {
   *kr {
     arg channel = 0, mul = 1.0, add = 0;

     ^this.multiNew('control', channel).madd(mul, add)
   }

    *rollKr {
      arg mul = 1.0, add = 0;
      ^this.kr(0, mul, add);
    }

    *pitchKr{
      arg mul = 1.0, add = 0;
      ^this.kr(1, mul, add);
    }

    *headingKr{
      arg mul = 1.0, add = 0;
      ^this.kr(2, mul, add);
    }
}
