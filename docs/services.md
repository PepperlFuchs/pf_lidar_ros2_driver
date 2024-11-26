## ROS Services

The ROS driver provides the following services. They use the sensor specific Pepperl+Fuchs scan data
protocol (PFSDP - [R2000](https://files.pepperl-fuchs.com/webcat/navi/productInfo/doct/doct3469g.pdf) /
[R2300](https://files.pepperl-fuchs.com/webcat/navi/productInfo/doct/doct7001b.pdf)) which is a simple 
command protocol using HTTP requests and responses. Please see the following detailed descriptions and 
examples to call these services. The examples are based on the assumption that the node name is pf_r2000.
Which parameters can be written with which values can be found in the PFSDP documents (R2x00 Ethernet 
communication protocol) linked above.

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

## Service type and interface:
The service type determines which data is used for the request and response of a service. The following command is an 
example for determining the type of the pfsdp_set_parameter service:
```
ros2 service type /pf_r2000/pfsdp_set_parameter
```
The return should be the following. Which is also the path to the file where the request and response data is described in detail:
```
pf_interfaces/srv/PfsdpSetParameter
```
To show the request and response interface/data for the pfsdp_set_parameter service the following command can be used:
```
ros2 interface show pf_interfaces/srv/PfsdpSetParameter
```
The return should be the following and shows the data name and type of the request above and of the response below the three dashes.
```
string name
string value
- - -
int32 error_code
string error_text
```

## Service execution status
All previously described services return at least the error code (error_code) and error text (error_text) in the response.
Both contain the status of the service execution. Error codes (data type int32) not equal to 0 indicate an error during service
execution. Error text (data type string) shows the status of the service execution as a description readable by humans.  
Exemplary the following response of the service pfsdp_get_parameter shows the error code, error text and as first return value
the read parameter vendor.
```
pf_interfaces.srv.PfsdpGetParameter_Response(value='Pepperl+Fuchs', error_code=0, error_text='success')
```
