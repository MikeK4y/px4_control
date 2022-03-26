#include "nmpc/master_control.h"

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "tf/transform_datatypes.h"

namespace px4_ctrl {
MasterControl::MasterControl(ros::NodeHandle &nh) {
  // Setup Subscribers
  drone_state_sub =
      nh.subscribe("/drone_state", 1, &MasterControl::droneStateCallback, this);

  // Setup Publishers
  trajectory_pub =
      nh.advertise<px4_control_msgs::Trajectory>("/drone_trajectory", 1);

  // Setup Service clients
  start_trajectory_serv =
      nh.serviceClient<std_srvs::Trigger>("/start_trajectory");
  enable_controller_serv =
      nh.serviceClient<std_srvs::SetBool>("/enable_controller");

  // Clear weights
  weights.clear();

  // Load parameters
  loadParameters();

  // Initialize acados NMPC
  traj_generation = new AcadosNMPC();
  if (traj_generation->initializeController(model_params) &&
      traj_generation->setWeighingMatrix(weights)) {
    ROS_INFO("NMPC Initialized\n");
  } else {
    ROS_ERROR("Failed to initialize Acados NMPC. Exiting\n");
    exit(1);
  }
}

MasterControl::~MasterControl() { delete traj_generation; };

void MasterControl::loadParameters() {
  // Create private nodehandle to load parameters
  ros::NodeHandle nh_pvt("~");

  nh_pvt.param("t_pitch", model_params.t_pitch, 1.0);
  nh_pvt.param("k_pitch", model_params.k_pitch, 1.0);

  nh_pvt.param("t_roll", model_params.t_roll, 1.0);
  nh_pvt.param("k_roll", model_params.k_roll, 1.0);

  std::vector<double> damping_coef;
  nh_pvt.getParam("damping_coef", damping_coef);
  model_params.damp_x = damping_coef[0];
  model_params.damp_y = damping_coef[1];
  model_params.damp_z = damping_coef[2];

  nh_pvt.param("k_thrust", model_params.k_thrust, 1.0);

  nh_pvt.param("gravity", model_params.gravity, -9.8066);

  std::vector<double> pos_w, vel_w, att_w;
  double yaw_rate_cmd_w, pitch_cmd_w, roll_cmd_w, thrust_cmd_w;

  nh_pvt.getParam("pos_w", pos_w);
  nh_pvt.getParam("vel_w", vel_w);
  nh_pvt.getParam("att_w", att_w);
  nh_pvt.getParam("yaw_rate_cmd_w", yaw_rate_cmd_w);
  nh_pvt.getParam("pitch_cmd_w", pitch_cmd_w);
  nh_pvt.getParam("roll_cmd_w", roll_cmd_w);
  nh_pvt.getParam("thrust_cmd_w", thrust_cmd_w);

  std::vector<double> setp_w, setp_m;
  nh_pvt.getParam("setpoint_world", setp_w);
  nh_pvt.getParam("setpoint_marker", setp_m);

  // Setpoint Init
  goal_world.pos_x = setp_w[0];
  goal_world.pos_y = setp_w[1];
  goal_world.pos_z = setp_w[2];
  goal_world.vel_x = 0.0;
  goal_world.vel_y = 0.0;
  goal_world.vel_z = 0.0;
  goal_world.q_roll = 0.0;
  goal_world.q_pitch = 0.0;
  goal_world.q_yaw = setp_w[3];

  goal_marker.pos_x = setp_m[0];
  goal_marker.pos_y = setp_m[1];
  goal_marker.pos_z = setp_m[2];
  goal_marker.vel_x = 0.0;
  goal_marker.vel_y = 0.0;
  goal_marker.vel_z = 0.0;
  goal_marker.q_roll = 0.0;
  goal_marker.q_pitch = 0.0;
  goal_marker.q_yaw = 0.0;

  setpoint_marker.x() = setp_m[0];
  setpoint_marker.y() = setp_m[1];
  setpoint_marker.z() = setp_m[2];

  // Cost function weights
  weights.push_back(pos_w[0]);
  weights.push_back(pos_w[1]);
  weights.push_back(pos_w[2]);
  weights.push_back(vel_w[0]);
  weights.push_back(vel_w[1]);
  weights.push_back(vel_w[2]);
  weights.push_back(att_w[0]);
  weights.push_back(att_w[1]);
  weights.push_back(att_w[2]);
  weights.push_back(yaw_rate_cmd_w);
  weights.push_back(pitch_cmd_w);
  weights.push_back(roll_cmd_w);
  weights.push_back(thrust_cmd_w);
}

void MasterControl::droneStateCallback(
    const px4_control_msgs::DroneStateMarker &msg) {
  // Send trajectory to first goal
  if (!traj_world) {
    trajectory_setpoint traj_start;
    traj_start.pos_x = msg.pose.position.x;
    traj_start.pos_y = msg.pose.position.y;
    traj_start.pos_z = msg.pose.position.z;
    traj_start.vel_x = msg.velocity.x;
    traj_start.vel_y = msg.velocity.y;
    traj_start.vel_z = msg.velocity.z;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.orientation, q);
    double y, p, r;
    tf::Matrix3x3(q).getEulerYPR(y, p, r);

    traj_start.q_roll = r;
    traj_start.q_pitch = p;
    traj_start.q_yaw = y;

    // Generate trajectory
    std::vector<trajectory_setpoint> trajectory;
    traj_generation->getTrajectory(trajectory, traj_start, goal_world);

    // Publish trajectory
    if (trajectory.size() > 0) {
      if (enableController(false)) {
        trajectory_pub.publish(setpointVectorToTrajMsg(trajectory));
        if (triggerTrajectoryTracking()) {
          if (enableController(true)) {
            traj_world = true;
            ROS_INFO("Trajectory to first goal sent and tracking enabled");
          }
        }
      }
    }
  } else {
    if (msg.marker_found.data) {
      // Send trajectory to second goal
      if (!traj_marker) {
        trajectory_setpoint traj_start;
        traj_start.pos_x = msg.pose.position.x;
        traj_start.pos_y = msg.pose.position.y;
        traj_start.pos_z = msg.pose.position.z;
        traj_start.vel_x = msg.velocity.x;
        traj_start.vel_y = msg.velocity.y;
        traj_start.vel_z = msg.velocity.z;

        tf::Quaternion q;
        tf::quaternionMsgToTF(msg.pose.orientation, q);
        double y, p, r;
        tf::Matrix3x3(q).getEulerYPR(y, p, r);

        traj_start.q_roll = r;
        traj_start.q_pitch = p;
        traj_start.q_yaw = y;

        // Get marker pose
        Eigen::Vector3d marker_pos(msg.marker_pose.position.x,
                                   msg.marker_pose.position.y,
                                   msg.marker_pose.position.z);
        Eigen::Quaterniond marker_q(
            msg.marker_pose.orientation.w, msg.marker_pose.orientation.x,
            msg.marker_pose.orientation.y, msg.marker_pose.orientation.z);

        // Get setpoint in world frame
        Eigen::Vector3d p_w_setpoint =
            marker_q.toRotationMatrix() * setpoint_marker + marker_pos;

        trajectory_setpoint traj_goal;
        traj_goal.pos_x = p_w_setpoint.x();
        traj_goal.pos_y = p_w_setpoint.y();
        traj_goal.pos_z = p_w_setpoint.z();
        traj_goal.vel_x = 0.0;
        traj_goal.vel_y = 0.0;
        traj_goal.vel_z = 0.0;

        tf::Quaternion q_m(marker_q.x(), marker_q.y(), marker_q.z(),
                           marker_q.w());
        tf::Matrix3x3(q_m).getEulerYPR(y, p, r);

        traj_goal.q_roll = 0.0;
        traj_goal.q_pitch = 0.0;
        traj_goal.q_yaw = y;

        // Generate trajectory
        std::vector<trajectory_setpoint> trajectory;
        traj_generation->getTrajectory(trajectory, traj_start, traj_goal);

        // Publish trajectory
        if (trajectory.size() > 0) {
          if (enableController(false)) {
            trajectory_pub.publish(setpointVectorToTrajMsg(trajectory));
            if (triggerTrajectoryTracking()) {
              if (enableController(true)) {
                marker_pos_set = marker_pos;
                traj_marker = true;
                ROS_INFO("Trajectory to marker goal sent and tracking enabled");
              }
            }
          }
        }
      } else {
        // Get marker position
        Eigen::Vector3d marker_pos(msg.marker_pose.position.x,
                                   msg.marker_pose.position.y,
                                   msg.marker_pose.position.z);
        double dx = (marker_pos_set - marker_pos).norm();

        // If the marker position has drifted more than 10cm set new trajectory
        if (dx > 0.1) {
          trajectory_setpoint traj_start;
          traj_start.pos_x = msg.pose.position.x;
          traj_start.pos_y = msg.pose.position.y;
          traj_start.pos_z = msg.pose.position.z;
          traj_start.vel_x = msg.velocity.x;
          traj_start.vel_y = msg.velocity.y;
          traj_start.vel_z = msg.velocity.z;

          tf::Quaternion q;
          tf::quaternionMsgToTF(msg.pose.orientation, q);
          double y, p, r;
          tf::Matrix3x3(q).getEulerYPR(y, p, r);

          traj_start.q_roll = r;
          traj_start.q_pitch = p;
          traj_start.q_yaw = y;

          // Get marker pose
          Eigen::Quaterniond marker_q(
              msg.marker_pose.orientation.w, msg.marker_pose.orientation.x,
              msg.marker_pose.orientation.y, msg.marker_pose.orientation.z);

          // Get setpoint in world frame
          Eigen::Vector3d p_w_setpoint =
              marker_q.toRotationMatrix() * setpoint_marker + marker_pos;

          trajectory_setpoint traj_goal;
          traj_goal.pos_x = p_w_setpoint.x();
          traj_goal.pos_y = p_w_setpoint.y();
          traj_goal.pos_z = p_w_setpoint.z();
          traj_goal.vel_x = 0.0;
          traj_goal.vel_y = 0.0;
          traj_goal.vel_z = 0.0;

          tf::Quaternion q_m(marker_q.x(), marker_q.y(), marker_q.z(),
                             marker_q.w());
          tf::Matrix3x3(q_m).getEulerYPR(y, p, r);

          traj_goal.q_roll = 0.0;
          traj_goal.q_pitch = 0.0;
          traj_goal.q_yaw = y;

          // Generate trajectory
          std::vector<trajectory_setpoint> trajectory;
          traj_generation->getTrajectory(trajectory, traj_start, traj_goal);

          // Publish trajectory
          if (trajectory.size() > 0) {
            if (enableController(false)) {
              trajectory_pub.publish(setpointVectorToTrajMsg(trajectory));
              if (triggerTrajectoryTracking()) {
                if (enableController(true)) {
                  marker_pos_set = marker_pos;
                  ROS_INFO(
                      "New trajectory to marker goal sent and tracking "
                      "enabled");
                }
              }
            }
          }
        }
      }
    }
  }
}

px4_control_msgs::Trajectory MasterControl::setpointVectorToTrajMsg(
    const std::vector<trajectory_setpoint> trajectory) {
  px4_control_msgs::Trajectory traj_msg;
  for (size_t i = 0; i < trajectory.size(); i++) {
    px4_control_msgs::Setpoint setpoint;
    setpoint.position.x = trajectory[i].pos_x;
    setpoint.position.y = trajectory[i].pos_y;
    setpoint.position.z = trajectory[i].pos_z;
    setpoint.velocity.x = trajectory[i].vel_x;
    setpoint.velocity.y = trajectory[i].vel_y;
    setpoint.velocity.z = trajectory[i].vel_z;
    setpoint.orientation.x = trajectory[i].q_roll;
    setpoint.orientation.y = trajectory[i].q_pitch;
    setpoint.orientation.z = trajectory[i].q_yaw;
    traj_msg.trajectory.push_back(setpoint);
  }
  return traj_msg;
}

bool MasterControl::triggerTrajectoryTracking() {
  std_srvs::Trigger srv_call;
  start_trajectory_serv.call(srv_call);
  return srv_call.response.success;
}

bool MasterControl::enableController(bool enable) {
  std_srvs::SetBool srv_call;
  srv_call.request.data = enable;
  enable_controller_serv.call(srv_call);
  return srv_call.response.success;
}
}  // namespace px4_ctrl

int main(int argc, char **argv) {
  ros::init(argc, argv, "master_controller_node");
  ros::NodeHandle nh;
  px4_ctrl::MasterControl mstr_ctrl(nh);

  ros::spin();

  return 0;
}