#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moiro_interfaces/srv/target_pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <vector>
#include <chrono>
#include <thread>
#include <stdexcept>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

geometry_msgs::msg::Pose target_pose;

void updateTargetPoseFromResponse(const geometry_msgs::msg::Pose &pose) {
  target_pose = pose;
  RCLCPP_INFO(LOGGER, "Target Pose: y=%f, z=%f", target_pose.position.y, target_pose.position.z);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_service", node_options);

  // MOIRO Target Pose Service Client
  auto target_pose_client = move_group_node->create_client<moiro_interfaces::srv::TargetPose>("vision/target_pose");

  // Moveit Initialization
  static const std::string PLANNING_GROUP = "mycobot_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  move_group.startStateMonitor();

  // 현재 상태 모니터를 위한 SingleThreadedExecutor 실행
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  // 서비스 클라이언트가 준비될 때까지 기다림
  while (!target_pose_client->wait_for_service(1s)) {
    RCLCPP_INFO(LOGGER, "서비스를 기다리는 중: vision/target_pose");
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(LOGGER, "ROS 중지 신호를 받음. 종료합니다.");
      executor_thread.join();
      return 1;
    }
  }

  while (rclcpp::ok()) {
    try {
      // 서비스 요청 생성
      auto request = std::make_shared<moiro_interfaces::srv::TargetPose::Request>();
      request->prepared = true;

      // 서비스 호출
      auto future = target_pose_client->async_send_request(request);
      RCLCPP_INFO(LOGGER, "서비스 호출: vision/target_pose");

      // 서비스 호출 결과 처리
      if (rclcpp::spin_until_future_complete(move_group_node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        geometry_msgs::msg::Pose human_pose;
        human_pose.position.y = response->y;
        human_pose.position.z = response->z;
        updateTargetPoseFromResponse(human_pose);
      } else {
        RCLCPP_ERROR(LOGGER, "서비스 호출 실패: vision/target_pose");
        continue;
      }

      if (target_pose.position.y != 0.0 || target_pose.position.z != 0.0) {
        geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(current_pose);

        geometry_msgs::msg::Pose eff_target_pose = target_pose;
        eff_target_pose.position.x = current_pose.position.x;
        eff_target_pose.orientation.w = 1.0;
        waypoints.push_back(eff_target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 10.0;
        const double eef_step = 0.03;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        RCLCPP_INFO(LOGGER, "계획 시각화 (카르테시안 경로) (%.2f%% 달성)", fraction * 100.0);

        // 경로 설정 및 실행
        robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), PLANNING_GROUP);
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        bool success = iptp.computeTimeStamps(rt, 1.0, 1.0);
        if (!success) {
          RCLCPP_WARN(LOGGER, "카르테시안 경로의 시간 매개변수화 실패");
        }

        rt.getRobotTrajectoryMsg(trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        move_group.execute(plan);

        // 현재 포즈 로그 출력
        geometry_msgs::msg::Pose updated_pose = move_group.getCurrentPose().pose;
        RCLCPP_INFO(LOGGER, "현재 포즈: x=%f, y=%f, z=%f", updated_pose.position.x, updated_pose.position.y, updated_pose.position.z);

        // target pose 초기화
        target_pose.position.x = 0;
        target_pose.position.y = 0;
        target_pose.position.z = 0;
      }

      std::this_thread::sleep_for(100ms); // 100ms 대기
    } catch (const std::exception &e) {
      RCLCPP_ERROR(LOGGER, "예외 발생: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(LOGGER, "알 수 없는 예외 발생");
    }
  }

  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}
