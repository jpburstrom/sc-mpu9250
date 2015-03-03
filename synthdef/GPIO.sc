GPIO : UGen {
   *kr {
     arg pin = 0, mul = 1.0, add = 0;

     ^this.multiNew('control', pin).madd(mul, add)
   }
}
