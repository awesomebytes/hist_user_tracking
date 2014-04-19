/*!
  File:         robot_wanderer_with_moving_goal.cpp
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

  This node enables the following and tracking of a moving goal.
  The free space is obtained thanks to the local costmap
  delivered by the move_base componnet.

  Many parameters enable to configure the behaviour of the robot.
  Most notably minimum and maximum speeds

  Parameters:
   * robot_frame_id
      [string] (default: "/base_link")
      The frame of the robot, normally attached to the base link.
   * static_frame_id
      [string] (default: "/odom")
      A static frame for the world, like /map or /odom .
   * inflated_obstacles_topic
      [string] (default: "/move_base/local_costmap/inflated_obstacles")
      Where to get the costmap in the local frame.
   * goal_pt_topic
      [string] (default: "/moving_goal")
      Where will be sent the actualizations for the moving goal.
   * cmd_vel_topic
      [string] (default: "/cmd_vel")
      Where to publish the speed orders that will move the base.
   * min_vel_lin
      [double] m.s-1 (default: .1)
      The minimum linear speed.
   * max_vel_lin
      [double] m.s-1 (default: .3)
      The maximum linear speed.
   * max_vel_ang
      [double] rad.s-1 (default: .5)
      The maximum absolute angular speed. The minimum angular speed is 0.
      The search domain is then: [-max_vel_ang .. max_vel_ang]
   * min_goal_distance
      [double] m (default: .6)
      The minimum distance between the robot center and the goal center
      When this distance is reached, the robot stopped.
      (do not forget that it includes the robot radius!)

  Subscriptions:
  * <inflated_obstacles_topic>
      [nav_msgs/GridCells]
      Where we get the local costmap.
  * <goal_pt_topic>
      [geometry_msgs/PoseStamped]
      Where to get the actualisations for the goal.

  Publications:
  * <cmd_vel_topic>
      [geometry_msgs::Twist]
      The orders sent to the base.
  * "/robot_wanderer_traj"
      [visualization_msgs::Marker]
      A marker to visualize the current trajectory.
  */

// uncomment to make the robot talk about its state
//#define USE_ETTS

// C
#include <signal.h>
// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
// ros utils
//#include "ros_utils/marker_utils.h"
//#include "ros_utils/costmap_utils.h"
//#include "ros_utils/odom_utils.h"
// hist_user_tracking
#include "conditional_particle_filter_laser_ros.h"

#ifdef USE_ETTS
// AD
#include "etts/skill_templates/ad_voice_skill/ad_voice_skill.h"
#endif // USE_ETTS

// TYPEDEFS
typedef conditional_particle_filter_laser::Pt2 Pt2;
struct Pose2 {
  Pt2 position;
  float yaw;
};

//! the publishers
ros::Publisher _vel_pub, _marker_pub;
//! the vizu marker
visualization_msgs::Marker _marker;

std::string _static_frame_id = "/odom";


/*
* Parameters of the robot
*/
std::string _robot_frame_id = "/base_link";
//! the maximal velocities, m/s or rad/s
double min_vel_lin = .1, max_vel_lin = .3, max_vel_ang = .5;
//! the current velocities, m/s or rad/s
float _vel_lin = 0.1, _vel_ang = 0.1;
//! the current pose of the robot
Pose2 _current_robot_pose;

// voice
#ifdef USE_ETTS
EttsApi etts_api;
CEventManager event_client;
CmemCortoPlazo mcp_client;
RobotConfig robot_config;
#endif // USE_ETTS

//! the costmap receiving
std::string _inflated_obstacles_topic = "/move_base/local_costmap/inflated_obstacles";
nav_msgs::GridCells _local_costmap;
//#define EMIT_LOCAL_COSTMAP_MARKER

//! where to get the goal
std::string _goal_pt_topic = "/moving_goal";
//! the current goal
Pose2 _goal;
/*! The minimum distance between the robot center and the goal center
      When this distance is reached, the robot stopped.
      (do not forget that it includes the robot radius!) */
double min_goal_distance = .6;
std::string cmd_vel_topic = "/cmd_vel";

tf::TransformListener* _tf_listener;
//! timer since last computed speed
TimerRos _speed_timer;
//! a maximum time before we choose some other speeds
static const float speed_timeout = 1;


/*
* Parameters for the simulation
*/
//! the forecast time in seconds
static const float TIME_PRED = 5;
//! the time step simulation
static const float DT = 0.2;
//! the number of tried trajectories
static const int MAX_TRIES = 1000;
std::vector<Pt2> _traj_buffer;


////////////////////////////////////////////////////////////////////////////////

/*!
  stop the robot
*/
void stop_robot() {
  _vel_lin = _vel_ang = 0;
  geometry_msgs::Twist out;
  _vel_pub.publish(out);
}


////////////////////////////////////////////////////////////////////////////////

/*!
  send a 0 speed to the robot and exit program
 \param param
    the exit code returned
*/
void stop_robot_and_exit(int param = 0) {
  ROS_INFO("stop_robot_and_exit()");
  stop_robot();
  delete _tf_listener;
  ros::shutdown();
  exit(param);
}

////////////////////////////////////////////////////////////////////////////////

inline float traj_mark(const float & curr_vel_lin, const float & curr_vel_ang) {
  return costmap_utils::trajectory_mark<Pt2>
      (_current_robot_pose.position, // Pt2
       _current_robot_pose.yaw, // float
       _goal.position, // Pt2
       _local_costmap, // nav_msgs::GridCells
       TIME_PRED, DT,
       curr_vel_lin, curr_vel_ang,
       _traj_buffer);
}

////////////////////////////////////////////////////////////////////////////////

void goal_callback(const geometry_msgs::PoseStampedConstPtr & pt3D) {
  //ROS_INFO_THROTTLE(1, "goal_callback()");
  //ROS_INFO("goal_callback()");

  if (pt3D->header.frame_id != _static_frame_id) {
    ROS_FATAL("The point frame_id '%s' is different from our static frame_id '%s'!",
              pt3D->header.frame_id.c_str(), _static_frame_id.c_str());
    stop_robot_and_exit(-1);
  }

  // do nothing if the point is set in (0, 0, 0)
  if (pt3D->pose.position.x == 0
      && pt3D->pose.position.y == 0
      && pt3D->pose.position.z == 0) {
    // first time we stop, say something
    if (_vel_lin != 0 || _vel_ang != 0) {
      ROS_INFO("There is no valid goal. Stopping the robot.");
#ifdef USE_ETTS
      etts_api.sayTextNL("|en:There is no goal.");
#endif // USE_ETTS
      stop_robot();
    }
    return;
  }

  // make a copy of the goal
  pt_utils::copy2(pt3D->pose.position, _goal.position);
  _goal.yaw = tf::getYaw(pt3D->pose.orientation);
  //_goal = *pt3D;

  // get the position of the robot in the static frame
  geometry_msgs::PoseStamped robot_pose_robot_frame, robot_pose_static_frame;
  try {
    robot_pose_robot_frame.header.frame_id = _robot_frame_id;
    robot_pose_robot_frame.header.stamp = ros::Time::now() - ros::Duration(1);
    robot_pose_robot_frame.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    _tf_listener->transformPose(_static_frame_id,  ros::Time(0),
                                robot_pose_robot_frame,
                                _static_frame_id,
                                robot_pose_static_frame);
  } catch (tf::ExtrapolationException e) {
    ROS_WARN_THROTTLE(2, "transform error:'%s'", e.what());
    return;
  }
  pt_utils::copy2(robot_pose_static_frame.pose.position,
                  _current_robot_pose.position);
  _current_robot_pose.yaw = tf::getYaw(robot_pose_static_frame.pose.orientation);
  //  ROS_INFO_THROTTLE(1, "Robot is in (%g, %g), yaw:%g in '%s'.",
  //                    _current_robot_pose.position.x, _current_robot_pose.position.y,
  //                    _current_robot_pose.yaw, _static_frame_id.c_str());

  // determine if we have reached the goal
  float dist_to_goal = geometry_utils::distance_points
      (_current_robot_pose.position, _goal.position);
  //ROS_INFO("dist_to_goal:%g", dist_to_goal);
  if (dist_to_goal < min_goal_distance) {
    ROS_INFO_THROTTLE(3, "Goal reached (dist:%g < min_goal_distance:%g) ! Stopping.",
                      dist_to_goal, min_goal_distance);
    if (_vel_lin != 0 || _vel_ang != 0) // say something when we stop
#ifdef USE_ETTS
      etts_api.sayTextNL("|en:Here I am.|es:He llegado.");
#endif // USE_ETTS
    // if yes, stop and return
    stop_robot();
    return;
  }

  /*
   * determine if needed to recompute the trajectory
   */
  bool want_recompute_speed = false;

  // recompute speeds if the last ones are too old
  if (!want_recompute_speed && _speed_timer.getTimeSeconds() > speed_timeout) {
    ROS_INFO("speed_timeout");
    want_recompute_speed = true;
  }

  // check if there will be a collision soon with the elected speeds
  // this comes included in
#if 0
  if (!want_recompute_speed &&
      !costmap_utils::is_trajectory_collision_free
      (_current_robot_pose.position, _current_robot_pose.yaw,
       _local_costmap, TIME_PRED, DT, _vel_lin, _vel_ang, _traj_buffer)) {
    ROS_INFO("coming_collision !");
    want_recompute_speed = true;
  } // end if not is_trajectory_collision_free()
#endif

  // check if we are getting far away from the goal
  if (!want_recompute_speed && traj_mark(_vel_lin, _vel_ang) > 1.5 * dist_to_goal) {
    ROS_INFO_THROTTLE(1, "getting far away (or maybe collision)");
    want_recompute_speed = true;
  } // end

  /*
   * recompute speeds when needed
   */
  if (want_recompute_speed) {
    float best_vel_lin = 0, best_vel_ang = 0,
        lowest_speed_grade = std::numeric_limits<float>::infinity();
    _speed_timer.reset();
    // choose a random speed and check it enables at least 1 second of movement
    for (int nb_tries = 0; nb_tries < MAX_TRIES; ++nb_tries) {
      // authorize on place rotations only after having tried many times
      float curr_min_vel_lin = (nb_tries < 100 ? min_vel_lin : 0);
      float curr_vel_lin = curr_min_vel_lin + drand48() * (max_vel_lin - curr_min_vel_lin);
      float curr_vel_ang = drand48() * 2 * max_vel_ang - max_vel_ang;
      float curr_grade = traj_mark(curr_vel_lin, curr_vel_ang);

#if 0 // export traj as marker
      marker_utils::list_points2_as_primitives
          (_marker, _traj_buffer, "traj_xy", 0.1, 0.03, 1, 0.5, 1, 1, _static_frame_id);
      _marker_pub.publish(_marker);
      ROS_INFO("trajectory_mark(vel_lin:%g, vel_ang:%g) : mark:%g",
               curr_vel_lin, curr_vel_ang, curr_grade);
      sleep(1);
#endif

      if (curr_grade < lowest_speed_grade) {
        lowest_speed_grade = curr_grade;
        best_vel_lin = curr_vel_lin;
        best_vel_ang = curr_vel_ang;
      }
    } // end loop nb_tries
    bool new_speeds_found = (lowest_speed_grade < std::numeric_limits<float>::infinity());

    if (!new_speeds_found) {
      ROS_ERROR("The robot is stuck! Stopping motion and exiting.");
#ifdef USE_ETTS
      etts_api.sayTextNL("|en:I am completely blocked.");
#endif // USE_ETTS
      stop_robot_and_exit(-1);
    } // end not new_speeds_found

    if (_vel_lin == 0 && _vel_ang == 0) {
#ifdef USE_ETTS
      etts_api.sayTextNL("|en:Let's go!|es:Adelante!");
#endif // USE_ETTS
    }
    _vel_lin = best_vel_lin;
    _vel_ang = best_vel_ang;
    ROS_INFO_THROTTLE(1, "Found the most suitable couple of speeds in %g ms, with a grade %g:"
                      "_vel_lin:%g, _vel_ang:%g",
                      _speed_timer.getTimeMilliseconds(), lowest_speed_grade, _vel_lin, _vel_ang);
    _speed_timer.reset();
  } // end if (want_recompute_speed)
  else {
    ROS_INFO_THROTTLE(1, "We are happy with the speeds _vel_lin:%g, _vel_ang:%g",
                      _vel_lin, _vel_ang);
  }

  // emit marker
  std::vector<Pt2> traj_xy;
  odom_utils::make_trajectory(_vel_lin, _vel_ang, traj_xy, TIME_PRED, DT, 0, 0, 0);
  marker_utils::list_points2_as_primitives(_marker, traj_xy, "traj_xy", 0.1, 0.03, 1, 0, 0);
  _marker_pub.publish(_marker);

  // publish the computed speed
  geometry_msgs::Twist out;
  out.linear.x = _vel_lin;
  out.angular.z = _vel_ang;
  ROS_INFO_THROTTLE(1, "Publishing _vel_lin:%g, _vel_ang:%g", _vel_lin, _vel_ang);
  _vel_pub.publish(out);
} // end goal_callback();

////////////////////////////////////////////////////////////////////////////////

void inflated_obstacles_callback(const nav_msgs::GridCellsConstPtr & msg) {
  //ROS_INFO("msg.header.frame_id:'%s'", msg->header.frame_id.c_str());
  if (msg->header.frame_id != _static_frame_id) {
    ROS_FATAL("The costmap frame_id '%s' is different from our static frame_id '%s'!",
              msg->header.frame_id.c_str(), _static_frame_id.c_str());
    stop_robot_and_exit(-1);
  }

  // keep a copy of the costmap
  _local_costmap = *msg;

#ifdef EMIT_LOCAL_COSTMAP_MARKER
  // publish cells centers
  std::vector<geometry_msgs::Point> cell_centers;
  for (unsigned int cell_idx = 0; cell_idx < msg->cells.size(); ++cell_idx) {
    const geometry_msgs::Point & curr_cell = msg->cells[cell_idx];
    cell_centers.push_back(geometry_msgs::Point(curr_cell.x, curr_cell.y));
  } // end loop cell_idx
  marker_utils::list_points2_as_primitives(_marker, cell_centers, "cell_centers",
                                           0.1, 0.05, 0, 1, 0, 1, static_frame_id);
  _marker_pub.publish(_marker);
#endif // EMIT_LOCAL_COSTMAP_MARKER
} // end inflated_obstacles_callback();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_wanderer_with_moving_goal");

  // stop the robot if signal received
  signal(SIGINT, stop_robot_and_exit);
  signal(SIGTERM, stop_robot_and_exit);
  signal(SIGKILL, stop_robot_and_exit);

#ifdef USE_ETTS
  // configure AD
  ArgsParser parser = AdVoiceSkill::test_prepair_parser(argc, argv);
  parser.parse();
  robot_config = RobotConfig::get_config_from_parsed_args_parser(parser);
  etts_api.set_pointers(&event_client, &mcp_client, &robot_config);
  etts_api.apply_options_from_args_parser(parser);
#endif // USE_ETTS

  //configure ROS
  ros::NodeHandle nh("~");
  _tf_listener = new tf::TransformListener();

  // get params
  nh.param<std::string>("robot_frame_id", _robot_frame_id, _robot_frame_id);
  nh.param<std::string>("static_frame_id", _static_frame_id, _static_frame_id);
  nh.param<std::string>("inflated_obstacles_topic",
                        _inflated_obstacles_topic,
                        _inflated_obstacles_topic);
  nh.param<std::string>("goal_pt_topic", _goal_pt_topic, _goal_pt_topic);
  nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, cmd_vel_topic);
  nh.param<double>("min_vel_lin", min_vel_lin, min_vel_lin);
  nh.param<double>("max_vel_lin", max_vel_lin, max_vel_lin);
  nh.param<double>("max_vel_ang", max_vel_ang, max_vel_ang);
  nh.param<double>("min_goal_distance", min_goal_distance, min_goal_distance);
  ROS_INFO("Getting inflated obstacles from '%s', emitting speeds to '%s'",
           _inflated_obstacles_topic.c_str(), cmd_vel_topic.c_str());

  // publishers
  _marker_pub = nh.advertise<visualization_msgs::Marker>("/robot_wanderer_traj", 1);
  _vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
  // subscribers
  ros::Subscriber inflated_sub = nh.subscribe
      (_inflated_obstacles_topic, 1, inflated_obstacles_callback);
  ros::Subscriber _goal_sub = nh.subscribe
      (_goal_pt_topic, 1, goal_callback);

  // spin
  ros::spin();

  stop_robot_and_exit();
  return 0;
}
