s.waitForBoot({
    {SinOsc.ar(AHRS.kr(0, 0.2, AHRS.kr(1, GPIO.kr(26, 0.2), 72))) }.play
}, 300, {"oops".println})
