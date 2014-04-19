/*!
  File:         conditional_particle_filter.h
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

  An implementation of
  "Conditional particle filters for simultaneous mobile robot
  localization and people-tracking"
  by Montemerlo, M.; Thrun, S.; Whittaker, W.;
  in 2002

  http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1013439&tag=1

  */

#ifndef CONDITIONALPARTICLEFILTER_H
#define CONDITIONALPARTICLEFILTER_H

// hist_user_tracking
#include "ros_utils/select_index_wth_probas.h"
#include "ros_utils/timer_ros.h"
#include "ros_utils/odom_utils.h"

namespace conditional_particle_filter_laser {

//! a probability
typedef float Proba;

//! the pose of the robot "r" : x, y, yaw
struct RobotPose {
  float x, y, yaw;
};

typedef odom_utils::FooRobotCommandOrder RobotCommandOrder;

//////////////////////////////////////////////////////////////////////////////

template<class SensorMeasurement, class PeoplePose>
class ConditionalParticleFilter {
public:
  /*! the world state "x" : contains the PeoplePose "y1..M",
    and also the RobotPose "r" */
  struct WorldState {
    RobotPose* robot_pose;
    std::vector<PeoplePose>* people_poses;
    //! ctor
    WorldState(RobotPose* _robot_pose, std::vector<PeoplePose>* _people_poses) :
      robot_pose(_robot_pose), people_poses(_people_poses) {}
  };

  //! all the particles [people_idx][robot_pose_part_idx][people_pose_part_idx]
  typedef std::vector< std::vector< std::vector<PeoplePose > > > ParticleVector3;


  //! number of particles for RobotPose estimation
  static const int N_r = 10;
  //! number of particles for PeoplePose estimation
  static const int N_y = 10;

  ////////////////////////////////////////////////////////////////////////////////

  //! ctor
  ConditionalParticleFilter() {
    timer.reset();
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! the likelihood of a sensor measurement depending of the world
      to implement */
  virtual Proba sensor_measurement_model(const SensorMeasurement & z_t,
                                         const WorldState & x_t)
  = 0;

  //////////////////////////////////////////////////////////////////////////////

  /*! robot motion given odometry data
      to implement */
  virtual RobotPose sample_robot_motion_model_law(const RobotCommandOrder & u_t,
                                                  const RobotPose & r_t_minus1,
                                                  const TimerRos::Time & dt_sec)
  = 0;

  //////////////////////////////////////////////////////////////////////////////

  /*! for instance brownian motion
      to implement */
  virtual PeoplePose sample_people_motion_model_law(const RobotCommandOrder & u_t,
                                                    const PeoplePose & y_t_minus1,
                                                    const TimerRos::Time & dt_sec)
  = 0;

  //////////////////////////////////////////////////////////////////////////////


  //! R_t : set of estimated RobotPose
  //! Y_t : for each estimated RobotPose, set of estimated PeoplePose for each person
  void algo(const RobotCommandOrder & u_t,
            const SensorMeasurement & z_t,
            const std::vector<RobotPose> & R_t_minus1,
            const ParticleVector3 & Y_t_minus1,
            std::vector<RobotPose> & R_t,
            ParticleVector3 & Y_t) {
    //! get the time elapsed
    TimerRos::Time time_elapsed = timer.getTimeSeconds();
    //! number of tracked persons
    int M = 1;
    //! copy all previous values
    R_t = R_t_minus1;
    Y_t = Y_t_minus1;

    //! contains the probability of the persons distribution for particle (i,j)
    Proba w2[N_r][N_y], w1[N_r];

    for (int i = 0; i < N_r; ++i) {
      // sample the new position of the robot
      const RobotPose & r_t_minus1 = R_t_minus1[i];
      R_t[i] = sample_robot_motion_model_law(u_t, r_t_minus1, time_elapsed);

      for (int j = 0; j < N_y; ++j) {
        for (int m = 0; m < M; ++m) {
          // sample the new position of the person "y_m_t_ij"
          const PeoplePose & y_m_t_minus1_ij = Y_t_minus1[i][j][m];
          Y_t[i][j][m]= sample_people_motion_model_law
              (u_t, y_m_t_minus1_ij, time_elapsed);
        } // end loop m
        // now, for each person, its particle position
        // have been sampled for this RobotPose

        WorldState x_ij_t(&R_t[i], &Y_t[i][j]);
        // find the likelihood of this particle
        w2[i][j] = sensor_measurement_model(z_t, x_ij_t);
      } // end loop j

      // cf http://www.cplusplus.com/reference/std/numeric/accumulate/
      Proba sum_w2_i = std::accumulate(w2[i], w2[i] + N_y, 0);
      // for all persons (m), Y_t[i][j][m]   <-   Y_t[i][ĸ][m]
      // (just choose the best particle)
      for (int j = 0; j < N_y; ++j) {
        unsigned int k =
            select_index_with_probas::select_given_sum(w2[i], w2[i] + N_y, sum_w2_i);
        for (int m = 0; m < M; ++m)
          Y_t[i][j][m] = Y_t[i][k][m];
      } // end loop j

      w1[i] = sum_w2_i;
    } // end loop i

    // fot all
    for (int i = 0; i < N_r; ++i) {
      unsigned int k =
          select_index_with_probas::select(w1, w1 + N_r);
      R_t[i] = R_t[k];
    } // end loop i
  } // end algo();

  //////////////////////////////////////////////////////////////////////////////

  //! the same as algo() except we use the stored values
  void algo_with_storage(const RobotCommandOrder & u_t,
                         const SensorMeasurement & z_t) {
    algo(u_t, z_t, _R_t_minus1, _Y_t_minus1, _R_t, _Y_t);
    // store the new results
    _R_t_minus1 = _R_t;
    _Y_t_minus1 = _Y_t;
  } // end algo();

protected:
  TimerRos timer;
  std::vector<RobotPose> _R_t_minus1, _R_t;
  ParticleVector3 _Y_t_minus1, _Y_t;

}; // end class ConditionalParticleFilter

} // end namesapce  conditional_particle_filter_laser

#endif // CONDITIONALPARTICLEFILTER_H
