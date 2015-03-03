# jackd -p32 -dalsa -dhw:1,0 -p1024 -n3 -s &


groups | grep i2c >/dev/null

if [[ $? -ne 0 ]] ; then
  echo 'not in i2c group! add to group with the following command'
  echo 'sudo usermod -a -G i2c,gpio ' $USER
  exit
fi

# need to sudo later, so get the password now

# use default audio card
jackd -p32 -dalsa -p1024 -n3 -s &

echo; echo; echo; echo staring scsynth

SC_JACK_DEFAULT_INPUTS="system:capture_1" SC_JACK_DEFAULT_OUTPUTS="system" scsynth -u 57110
