/* M4-ATX diagnostics and configuration node
 *
 * LGPL (c) 2010 Ken Tossell <ktossell@umd.edu>
 */

#include "ros/ros.h"
#include "m4atx_battery_monitor/PowerReading.h"
#include "std_msgs/String.h"
#include <usb.h>

extern "C" {
  #include "m4atx_battery_monitor/m4api.h"
}

int main (int argc, char **argv) 
{
	ros::init(argc, argv, "m4atx");

	ros::NodeHandle node;
	ros::Publisher diag_pub, sound_pub;
	double diag_frequency;
	double input_nominal;
	double battery_dead_voltage;
	ros::Time next_check = ros::Time::now();
	
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
	private_node_handle_.param<double>("battery_dead_voltage", battery_dead_voltage, 10.5);

	diag_pub = node.advertise<m4atx_battery_monitor::PowerReading>("battery_status_m4atx", 1);
	sound_pub = node.advertise<std_msgs::String>("say", 1);

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
		reading.volts_read_item.push_back("VIN");
		reading.volts_read_value.push_back(diag.vin);
		reading.volts_read_item.push_back("12V");
		reading.volts_read_value.push_back(diag.v12);
		reading.volts_read_item.push_back("5V");
		reading.volts_read_value.push_back(diag.v5);
		reading.volts_read_item.push_back("3.3V");
		reading.volts_read_value.push_back(diag.v33);
		reading.volts_read_item.push_back("VIGN");
		reading.volts_read_value.push_back(diag.vign);
		
		reading.volts_full_item.push_back("VIN");
		reading.volts_full_value.push_back(input_nominal);
		reading.volts_full_item.push_back("12V");
		reading.volts_full_value.push_back(12.0);
		reading.volts_full_item.push_back("5V");
		reading.volts_full_value.push_back(5.0);
		reading.volts_full_item.push_back("3.3V");
		reading.volts_full_value.push_back(3.3);

		reading.temperature.push_back(diag.temp);
		
		double soc = (diag.vin - battery_dead_voltage) / (input_nominal - battery_dead_voltage);
		soc *= 100;
		
		reading.input_soc = soc;
		
		reading.header.stamp = ros::Time::now();

		diag_pub.publish(reading);	
		
		if (ros::Time::now() > next_check)
		{
			std_msgs::String message; 
			if(soc < 5)
			{	
				next_check = ros::Time::now() + ros::Duration(5.0*60); //5 min 
				message.data = "Computer battery has 5 percent remaining";
				sound_pub.publish(message);
			}else if(soc < 10)
			{
				next_check = ros::Time::now() + ros::Duration(10.0*60); //10 min 
				message.data = "Computer battery has 10 percent remaining";
				sound_pub.publish(message);
			} else if(soc < 20)
			{
				next_check = ros::Time::now() + ros::Duration(15.0*60); //15 min
				message.data = "Computer battery has 20 percent remaining";
				sound_pub.publish(message);
			} else 
			{
				next_check = ros::Time::now() + ros::Duration(20.0*60); //20 min 	
			}
		}
		
		loop_rate.sleep();
	}
	
	//close the usb handle
	usb_close(dev);	
	
	return 0;
}

