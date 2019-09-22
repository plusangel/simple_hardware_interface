# hardware interface for sonar

## IOs

It reads input from topic **sonar_velocity** to forward it as PWM value (format: Int16)
It publishes to topic **encoder** the encoder ticks (format: Float32)

## Execution

Please use:

```
roslaunch hardware_interface_sonar hardware_interface.launch
```

## Tests
We can the test_publisher to send a continues steam of velocity data.
```
rosrun hardware_interface_sonar test_blisher
```
In case that we want a constant number please edit accordingly.


## Comments

The board is assumed to be found on /dev/ttyACM0. In different case just edit the config file.
Communication runs at 10Hz (100ms intervals).

For further changes please talk to author.


## Documentation
The project is commented using Doxygen. Please make *doc/create_doc.sh* executable
(chmod +x create_doc.sh) and run it. After that, index.html inside the doc folder in the home page of the
documentation site.

## Athor

[Angelos Plastropoulos](mailto:angelos.plastropoulos@gmail.com)

