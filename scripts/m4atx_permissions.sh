#!/bin/bash

# Script for the M4ATX power supply to ensure proper permissions when executing the ROS node
BUS_NUM=$((lsusb | grep "04d8:d001 Microchip Technology, Inc.") | awk '{print $2}')
DEV_NUM=$(((lsusb | grep "04d8:d001 Microchip Technology, Inc.") | awk '{print $4}') | cut -c 1-3)

LOCATION="/dev/bus/usb/${BUS_NUM}/${DEV_NUM}"
chmod a+rw ${LOCATION}
