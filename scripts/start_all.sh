# jackd -p32 -dalsa -dhw:1,0 -p1024 -n3 -s &

groups | grep i2c >/dev/null

if [[ $? -ne 0 ]] ; then
  echo 'not in i2c group! add to group with the following command'
  echo 'sudo usermod -a -G i2c,gpio ' $USER
  exit
fi

sudo killall scsynth jackd

# need to sudo later, so get the password now
sudo echo "starting scsynth"

# use default audio card
jackd -p32 -dalsa -p1024 -n3 -s &

# export pin 26 and set it to pull-up
sudo gpio -g  mode 26 up
sudo gpio -g  mode 26 in
sudo gpio -g  mode 26 up
sudo gpio export 26 in
sudo chgrp gpio /sys/class/gpio/gpio26/value

# SC_JACK_DEFAULT_INPUTS="system:capture_1" SC_JACK_DEFAULT_OUTPUTS="system" sudo scsynth -u 57110 -U /home/ccrma/.local/supercollider/extensions:/usr/local/lib/SuperCollider/plugins
SC_JACK_DEFAULT_INPUTS="system:capture_1" SC_JACK_DEFAULT_OUTPUTS="system" scsynth -u 57110
