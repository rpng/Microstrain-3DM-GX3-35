# microstrain_3dm_gx3_35

* IMU driver for the Microstrain 3DM-GX3®-35.
* Implementing MIP protocol communication.
* GPS driver are also included here (under development).
* Original package got from https://github.com/udrg/microstrain_comm
* Built with ROS indigo on Ubuntu 14.04 LTS

### Dependencies:
* GLIB2
* GTHREAD2
* libbot (already included)

### Parameters:
* `"verbose"`: default false, (if true packet bits are printed)
* `"debug"`: default false, (if true debug messages are printed)
* `"com_port"`: default "", (custom serial port, default scans /dev/ttyACM*)
* `"rate"`: default "low", ("high"=1000hz, "medium"=500hz, "low"=100hz) (now 100Hz works)
* `"init"`: default "yes", (do initialization, such as settiing data format etc)
