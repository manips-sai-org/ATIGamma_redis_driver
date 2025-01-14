# ATIGamma redis driver for SAI

This is a driver for the Ethernet version of the ATI Gamma 6 axis Froce/Torque sensor for use with SAI.

## Build instructions

This driver depends on [SaiCommon](https://github.com/manips-sai-org/sai-common). You need to have SaiCommon installed before compiling it. If you installed [SAI](https://github.com/manips-sai-org/SAI), SaiCommon is already installed on your system and you can proceed.

Make a build folder and compile the program using cmake:

```
mkdir build && cd build
cmake .. && make
```

## Usage

The driver will publish the sensed force and moment that the sensor applies to its environment to redis in two keys, one for the force, the other for the moment. The keys are of the form `<prefix>::sensors::<robot_name>::ft_sensor::<link_name>::force` and `<prefix>::sensors::<robot_name>::ft_sensor::<link_name>::moment`, where the prefix, robot name and link name are defined in the config file and must match the prefix, robot name and link name of the SAI controller motion force task that this sensor is used for.

The config file must be in the folder `config_folder`, and is an xml file containing one tag `saiATIGammaDriverConfig` with the following attributes:

- redisPrefix: the prefix for redis keys
- robotName: the name of the robot on which the sensor is attached (used to construct the redis key)
- linkName: the name of the link to which the sensor is attached (used to construct the redis key)
- sensorIpAddress: the ip address of the sensor Net box
- useFilter: whether to implement a second order butterworth filter on the sensor signal (in addition to the sensor built in filter)
- normalizedFilterCutoff: The normalized cutoff fewquency of the filter if used (between 0 and 0.5)

launch the driver from the build folder:
```
./ATIGamma_redis_driver [(optionnal) config_file_name]
```

If no config file name is provided, the `default_config.xml` is used