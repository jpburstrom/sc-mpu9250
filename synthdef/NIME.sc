s.waitForBoot({
  {
      var frames = 44100 * 60 * 3,
          b = Buffer.alloc(s, frames, 1), // a 3-minute, 1 channel buffer
          offset = AHRS.rollKr(0, 1/360, 0.5);

      // record into the buffer somewhere depending on AHRS data
      // RecordBuf.ar(SoundIn.ar(0), b, offset * frames, loop:1, run:GPIO.kr(26, -2 + 1, 1), trigger:Impulse.kr(30));
      RecordBuf.ar(SoundIn.ar(0), b, offset * frames, loop:1, run:1, trigger:Impulse.kr(30));

      // granulate out of the buffer at the same position
       GrainBuf.ar(1, Impulse.kr(10), 0.5, b, offset);
  }.play;
}, 300, {"oops".println})
