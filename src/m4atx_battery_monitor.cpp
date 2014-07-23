/*!
 * M4-ATX diagnostics and configuration node.
 *
 * LGPL (c) 2010 Ken Tossell <ktossell@umd.edu>
 *
 * \author Ken Tossell <ktossell@umd.edu>
 * \author Chris Dunkers, WPI - cmdunkers@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 */

#include <ros/ros.h>
#include <m4atx_battery_monitor/PowerReading.h>
#include <usb.h>

extern "C"
{
#include "m4atx_battery_monitor/m4api.h"
}

/*!
 * Creates and runs the m4atx_battery_monitor node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly, or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize the node
  ros::init(argc, argv, "m4atx_battery_monitor");

  // main node handle
  ros::NodeHandle node;
  ros::Time next_check = ros::Time::now();

  // attempt to initialize the USB device
  struct usb_dev_handle *dev = m4Init();
  if (!dev)
  {
    ROS_ERROR("Could not initialize the M4-ATX.");
    return EXIT_FAILURE;
  }

  // grab the parameters
  ros::NodeHandle private_nh("~");
  double diag_frequency;
  private_nh.param<double>("diag_frequency", diag_frequency, 1.0);
  double input_nominal;
  private_nh.param<double>("input_nominal", input_nominal, 13.8);
  double battery_dead_voltage;
  private_nh.param<double>("battery_dead_voltage", battery_dead_voltage, 10.5);

  // setup the publishers
  ros::Publisher diag_pub = node.advertise<m4atx_battery_monitor::PowerReading>("battery_status_m4atx", 1);

  // main publish loop
  ros::Rate loop_rate(diag_frequency);
  while (ros::ok())
  {
    m4Diagnostics diag;
    m4atx_battery_monitor::PowerReading reading;

    // attempt to red the battery
    if (m4GetDiag(dev, &diag))
    {
      ROS_ERROR("Error reading from the M4-ATX.");
      return EXIT_FAILURE;
    }

    // parse the feedback
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

    // calculate the soc value
    double soc = (diag.vin - battery_dead_voltage) / (input_nominal - battery_dead_voltage);
    soc *= 100;
    reading.input_soc = soc;
    reading.header.stamp = ros::Time::now();

    // publish the feedback
    diag_pub.publish(reading);

    // check if we need to say something to the user
    if (ros::Time::now() > next_check)
    {
      if (soc < 5)
      {
        next_check = ros::Time::now() + ros::Duration(5.0 * 60); //5 min
        system("espeak \"Computer battery has 5 percent remaining.\"");
      }
      else if (soc < 10)
      {
        next_check = ros::Time::now() + ros::Duration(10.0 * 60); //10 min
        system("espeak \"Computer battery has 10 percent remaining.\"");
      }
      else if (soc < 20)
      {
        next_check = ros::Time::now() + ros::Duration(15.0 * 60); //15 min
        system("espeak \"Computer battery has 20 percent remaining.\"");
      }
      else
      {
        next_check = ros::Time::now() + ros::Duration(20.0 * 60); //20 min
      }
    }

    loop_rate.sleep();
  }

  //close the USB handle
  usb_close(dev);

  return EXIT_SUCCESS;
}
