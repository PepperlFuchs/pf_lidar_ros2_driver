## ROS Services

The ROS driver provides the following services. They use the sensor specific Pepperl+Fuchs scan data
protocol (PFSDP) which is a simple command protocol using HTTP requests and responses. Please see the
following detailed descriptions and examples to call these services. The examples are based on the
assumption that the node name is pf_r2000.

**List parameters**  
The service pfsdp_list_parameters returns a list of all available global sensor parameters.
```
ros2 service call /pf_r2000/pfsdp_list_parameters pf_interfaces/srv/PfsdpListParameters
```

**Get parameter**  
The service pfsdp_get_parameter reads the current value of one global sensor parameters.
```
ros2 service call /pf_r2000/pfsdp_get_parameter pf_interfaces/srv/PfsdpGetParameter '{name: user_tag}'
```

**Set parameter**  
Using the service pfsdp_set_parameter the value of any write-accessible global sensor parameter can
be changed.
```
ros2 service call /pf_r2000/pfsdp_set_parameter pf_interfaces/srv/PfsdpSetParameter '{name: user_tag, value: test123}'
```

**Reset parameter**  
The service pfsdp_reset_parameter resets one global sensor parameter to the factory default value.
```
ros2 service call /pf_r2000/pfsdp_reset_parameter pf_interfaces/srv/PfsdpResetParameter '{name: user_tag}'
```

**List I/Q parameters**  
The service pfsdp_list_iq_parameters is similar to the generic list parameters service but returns
all parameters related to the switching input/output channels I/Qn.
```
ros2 service call /pf_r2000/pfsdp_list_iq_parameters pf_interfaces/srv/PfsdpListIqParameters
```

**Get I/Q parameter**  
The service pfsdp_get_iq_parameter is similar to the generic pfsdp_get_parameter service but operates
on parameters related to the switching input/output channels I/Qn. The service returns the current
value of one parameter.
```
ros2 service call /pf_r2000/pfsdp_get_iq_parameter pf_interfaces/srv/PfsdpGetIqParameter '{name: iq2_polarity}'
```

**Set I/Q parameter**  
The service pfsdp_set_iq_parameter is similar to the generic pfsdp_set_parameter service but operates
on parameters related to the switching input/output channels I/Qn. Using the service pfsdp_set_iq_parameter
the value of any write-accessible I/Q parameter can be modified.
```
ros2 service call /pf_r2000/pfsdp_set_iq_parameter pf_interfaces/srv/PfsdpSetIqParameter '{name: iq2_polarity, value: active_low}'
```

**Get protocol info**  
The service pfsdp_get_protocol_info returns basic version information on the communication protocol.
```
ros2 service call /pf_r2000/pfsdp_get_protocol_info pf_interfaces/srv/PfsdpGetProtocolInfo
```

**Reboot device**  
The service pfsdp_reboot_device triggers a soft reboot of the sensor firmware.
```
ros2 service call /pf_r2000/pfsdp_reboot_device pf_interfaces/srv/PfsdpRebootDevice
```

**Factory reset**  
The service pfsdp_factory_reset performs a complete reset of all sensor settings to factory defaults
and reboots the device.
```
ros2 service call /pf_r2000/pfsdp_factory_reset pf_interfaces/srv/PfsdpFactoryReset
```
