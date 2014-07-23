m4atx_battery_monitor [![Build Status](https://api.travis-ci.org/WPI-RAIL/m4atx_battery_monitor.png)](https://travis-ci.org/WPI-RAIL/m4atx_battery_monitor)
=====================

#### Battery Monitor for the M4-ATX Power Module
For full documentation, see [the ROS wiki](http://ros.org/wiki/m4atx_battery_monitor).

The node reads in and publishes the status of the battery supply. It also verbalizes the battery low warnings using `espeak`.

### Setup
`$ lsusb`

Find Microchip Technology, Inc. and mark down the bus and device numbers.

`$ sudo chmod a+rw /dev/bus/usb/[bus #]/[device #]`

Now the node will connect and run properly.

### License
For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
