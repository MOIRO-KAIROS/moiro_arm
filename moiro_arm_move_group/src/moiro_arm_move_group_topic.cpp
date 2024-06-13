#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <vector>
#include <chrono>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

geometry_msgs::msg::Pose target_pose;

void targetPoseCallback(const geometry_msgs::msg::Pose &msg)
{
  target_pose = msg;
  RCLCPP_INFO(LOGGER, "Target Pose: x=%f, y=%f, z=%f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
  auto target_pose_sub = move_group_node->create_subscription<geometry_msgs::msg::Pose>("target_pose", 10, targetPoseCallback);

  // Moveit Initialization
  static const std::string PLANNING_GROUP = "moiro_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  move_group.startStateMonitor();

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  while (rclcpp::ok())
  {
    if (target_pose.position.y != 0.0 || target_pose.position.z != 0.0)
    {
      geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
      geometry_msgs::msg::Pose base_plate_pose = move_group.getCurrentPose("base_plate").pose;
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(current_pose);

      geometry_msgs::msg::Pose eff_target_pose;
      eff_target_pose.position.x = current_pose.position.x;
      eff_target_pose.position.y = base_plate_pose.position.y + target_pose.position.y;
      eff_target_pose.position.z = base_plate_pose.position.z + target_pose.position.z;
      eff_target_pose.orientation.w = 1.0;
      RCLCPP_INFO(LOGGER, "Eff Target Pose: x=%f, y=%f, z=%f", eff_target_pose.position.x, eff_target_pose.position.y, eff_target_pose.position.z);
      waypoints.push_back(eff_target_pose);

      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 10.0;
      const double eef_step = 0.03;
      double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      RCLCPP_INFO(LOGGER, "Visualizing plan (cartesian path) (%.2f%% achieved)", fraction * 100.0);

      // Trajectory 설정 및 실행
      robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), PLANNING_GROUP);
      rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      bool success = iptp.computeTimeStamps(rt, 1.0, 1.0);
      if (!success)
      {
        RCLCPP_WARN(LOGGER, "Time parametrization for the Cartesian path failed.");
      }

      rt.getRobotTrajectoryMsg(trajectory);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      move_group.execute(plan);

      // 현재 포즈 로그 출력
      current_pose = move_group.getCurrentPose().pose;
      RCLCPP_INFO(LOGGER, "Current Pose: x=%f, y=%f, z=%f", current_pose.position.x, current_pose.position.y, current_pose.position.z);

      // target pose 초기화
      target_pose.position.x = 0;
      target_pose.position.y = 0;
      target_pose.position.z = 0;
    }

    std::this_thread::sleep_for(100ms); // 100ms 대기
  }

  rclcpp::shutdown();
  return 0;
}