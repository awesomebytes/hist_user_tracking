/*!
  File:         conditional_particle_filter_laser_ros.h
  Author:       Arnaud Ramey <arnaud.a.ramey@gmail.com>
  Date:         2012/1
  ______________________________________________________________________________

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  ______________________________________________________________________________

  */

#ifndef CONDITIONAL_PARTICLE_FILTER_LASER_ROS_H
#define CONDITIONAL_PARTICLE_FILTER_LASER_ROS_H

#include "conditional_particle_filter_laser.h"
// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>

//#include "ros_utils/laser_utils.h"

namespace conditional_particle_filter_laser {

////////////////////////////////////////////////////////////////////////////////

class ConditionalParticleFilterLaserRos : public ConditionalParticleFilterLaser {
public:
  typedef geometry_msgs::TwistStamped             VelCmd;
  typedef geometry_msgs::TwistStampedConstPtr     VelCmdConstPtr;
  typedef sensor_msgs::LaserScan                  LaserMsg;
  typedef sensor_msgs::LaserScanConstPtr          LaserMsgConstPtr;
  typedef message_filters::sync_policies::ApproximateTime<LaserMsg, VelCmd> DataSyncPolicy;

private:
  ros::NodeHandle nh;
  message_filters::Subscriber<LaserMsg> _laser_subscriber;
  message_filters::Subscriber<VelCmd> _com_vel_subscriber;

public:
  //////////////////////////////////////////////////////////////////////////////

  //! ctor
  ConditionalParticleFilterLaserRos() :
    _laser_subscriber(nh, "/scan_filtered", 1),
    _com_vel_subscriber(nh, "/cmd_vel_stamped", 1)
  {
    // ApproximateTime synchronizer
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    //policy.setMaxIntervalDuration (ros::Duration(30.f / 1000)); // max package of 30ms
    message_filters::Synchronizer<DataSyncPolicy> sync
        (DataSyncPolicy(1), _laser_subscriber, _com_vel_subscriber);

    sync.registerCallback
        (boost::bind
         (&ConditionalParticleFilterLaserRos::data_callback, this, _1, _2));

  }

  ////////////////////////////////////////////////////////////////////////////////

  //! this function is called each time an image is received
  void data_callback(const LaserMsgConstPtr & laser_msg,
                     const VelCmdConstPtr & cmd_vel_msg) {
    ROS_INFO("data_callback()");
    // get the robot order
    RobotCommandOrder u_t;
    pt_utils::copy3(cmd_vel_msg->twist.linear, u_t.linear);
    pt_utils::copy3(cmd_vel_msg->twist.angular, u_t.angular);

    // build the laser data
    SensorMeasurement z_t;
    laser_utils::convert_sensor_data_to_xy(laser_msg, z_t);

    // call the algorithm
    algo_with_storage(u_t, z_t);
  } // end data_callback();

private:

}; // end class ConditionalParticleFilterLaserRos

} // end namesapce  conditional_particle_filter_laser

#endif // CONDITIONAL_PARTICLE_FILTER_LASER_ROS_H
