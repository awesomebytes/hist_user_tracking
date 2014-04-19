/*!
  File:         conditional_particle_filter_laser.h
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

#ifndef CONDITIONNAL_PARTICLE_FILTER_LASER_H
#define CONDITIONNAL_PARTICLE_FILTER_LASER_H

#include <conditional_particle_filter.h>
#include "utils/geom/geometry_utils.h"

namespace conditional_particle_filter_laser {

//! a 2D point
typedef geometry_utils::FooPoint2f Pt2;

//! the sensor data "y" : the laser range.
typedef std::vector<Pt2> SensorMeasurement;

//! the pose of the human "y" : x, y, yaw
struct PeoplePose {
  float x, y, yaw;
};

////////////////////////////////////////////////////////////////////////////////

class ConditionalParticleFilterLaser :
    public ConditionalParticleFilter<SensorMeasurement, PeoplePose> {
  //////////////////////////////////////////////////////////////////////////////

  /*! the likelihood of a sensor measurement model depending of the world
      to implement */
  virtual Proba sensor_measurement_model(const SensorMeasurement & z_t,
                                         const WorldState & x_t)
  {


    return 0;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! robot motion given odometry data
      to implement */
  virtual RobotPose sample_robot_motion_model_law(const RobotCommandOrder & u_t,
                                                  const RobotPose & r_t_minus1,
                                                  const TimerRos::Time & dt_sec)
  {
    // odometry update
    RobotPose r_t = r_t_minus1;
    odom_utils::update_pos_rot(r_t.x, r_t.y, r_t.yaw, u_t, dt_sec);
    return r_t;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! for instance brownian motion
      to implement */
  virtual PeoplePose sample_people_motion_model_law(const RobotCommandOrder & u_t,
                                                    const PeoplePose & y_t_minus1,
                                                    const TimerRos::Time & dt_sec)
  {
    // odometry update
    PeoplePose y_t = y_t_minus1;
    odom_utils::update_pos_rot(y_t.x, y_t.y, y_t.yaw, u_t, dt_sec);

    // brownian move
    // TODO

    return y_t;
  }

}; // end class ConditionalParticleFilterLaser

} // end namesapce  conditional_particle_filter_laser

#endif // CONDITIONNAL_PARTICLE_FILTER_LASER_H
