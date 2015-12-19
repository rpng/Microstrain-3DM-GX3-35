# microstrain_3dm_gx3_35

* A ROS driver for the Microstrain 3DM-GX3®-35 implementing MIP protocol.
* Driver for GPS component is also included here (under development).
* Built with ROS indigo on Ubuntu 14.04 LTS
* Original package got from https://github.com/udrg/microstrain_comm

### Dependencies:
* GLIB2
* GTHREAD2
* libbot (already included)

### Parameters:
* `"verbose"`: default false, (if true packet bits are printed)
* `"debug"`: default false, (if true debug messages are printed)
* `"init"`: default false, (if true do initialization, such as settiing data format, etc.)
* `"com_port"`: default " ", (name of the serial port designated by user)
