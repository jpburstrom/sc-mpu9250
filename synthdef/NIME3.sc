s.waitForBoot({
    {SinOsc.ar(AHRS.kr(0, 2, GPIO.kr(26, 100, 400)))}.play
}, 300, {"oops".println})
