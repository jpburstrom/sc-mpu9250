# jackd -p32 -dalsa -dhw:1,0 -p1024 -n3 -s &

# use default audio card
jackd -p32 -dalsa -p1024 -n3 -s &
SC_JACK_DEFAULT_INPUTS="system:capture_1" SC_JACK_DEFAULT_OUTPUTS="system" scsynth -u 57110 &
