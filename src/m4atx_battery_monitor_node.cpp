/* M4-ATX diagnostics and configuration node
 *
 * LGPL (c) 2010 Ken Tossell <ktossell@umd.edu>
 */

#include "ros/ros.h"
#include "m4atx_battery_monitor/PowerReading.h"

extern "C" {
  #include "m4atx_battery_monitor/m4api.h"
}

int main (int argc, char **argv) 
{
	ros::init(argc, argv, "m4atx");

	ros::NodeHandle node;
	ros::Publisher diag_pub;
	double diag_frequency;
	double input_nominal;

	struct usb_dev_handle *dev;

	dev = m4Init();

	if (!dev) 
	{
		perror("Initializing M4-ATX");
		return -1;
	}

	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param<double>("diag_frequency", diag_frequency, 1.0);
	private_node_handle_.param<double>("input_nominal", input_nominal, 13.8);

	diag_pub = node.advertise<m4atx_battery_monitor::PowerReading>("m4atx_battery_status", 1);

	ros::Rate loop_rate(diag_frequency);
	while (ros::ok()) 
	{
		m4Diagnostics diag;
		m4atx_battery_monitor::PowerReading reading;

		if (m4GetDiag(dev, &diag)) 
		{
			perror("Reading from M4-ATX");
			return -1;
		}

		reading.volts_read.push_back(diag.vin);
		reading.volts_read.push_back(diag.v12);
		reading.volts_read.push_back(diag.v5);
		reading.volts_read.push_back(diag.v33);

		reading.volts_full.push_back(input_nominal);
		reading.volts_full.push_back(12.0);
		reading.volts_full.push_back(5.0);
		reading.volts_full.push_back(3.3);

		reading.temperature.push_back(diag.temp);

		reading.header.stamp = ros::Time::now();

		diag_pub.publish(reading);

		loop_rate.sleep();
	}

	return 0;
}

