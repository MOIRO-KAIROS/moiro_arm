#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moiro_interfaces/srv/target_pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <vector>
#include <math.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

struct HumanPose {
    bool valid = false;
    std::tuple<double, double, double> goal; // (x, y, z) 좌표
};

HumanPose p;

class MOIROARMMoveGroup : public rclcpp::Node {
public:
    MOIROARMMoveGroup() : Node("moiro_arm_move_group_service_class"), move_group_(std::shared_ptr<rclcpp::Node>(this), PLANNING_GROUP) {
        RCLCPP_INFO(this->get_logger(), "Initialized node");

        // Moveit 초기화
        move_group_.startStateMonitor();
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized");
        RCLCPP_INFO(this->get_logger(), "Reference frame: %s", move_group_.getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_.getEndEffectorLink().c_str());

        // Service Client 생성
        srv_client = this->create_client<moiro_interfaces::srv::TargetPose>("vision/target_pose");

        // 주기적으로 check_and_request 호출s
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&MOIROARMMoveGroup::check_and_request, this)
        );
    }

    void check_and_request() {
        RCLCPP_INFO(this->get_logger(), "Check and request");
        if (!p.valid) {
            request_target();
        }
    }

    void request_target() {
        RCLCPP_INFO(this->get_logger(), "Request target pose");
        auto request = std::make_shared<moiro_interfaces::srv::TargetPose::Request>();
        request->prepared = true;
        auto future = srv_client->async_send_request(
            request, std::bind(&MOIROARMMoveGroup::handle_response, this, std::placeholders::_1)
        );
    }

    void handle_response(rclcpp::Client<moiro_interfaces::srv::TargetPose>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "Handle response");
        try {
            auto response = future.get();
            p.goal = std::make_tuple(response->x, response->y, response->z);
            p.valid = true;
            RCLCPP_INFO(this->get_logger(), "Lost Human: %d", (response->status));

            RCLCPP_INFO(this->get_logger(), "Target Pose: x=%f, y=%f, z=%f", std::get<0>(p.goal), std::get<1>(p.goal), std::get<2>(p.goal));

            if (p.valid && response->status) {
                MyCobotRun();
            }
            p.valid = false;
        } catch (const std::exception &e) {
            p.valid = false;
            RCLCPP_ERROR(this->get_logger(), "Failed to get target pose: %s", e.what());
        }
    }

    void MyCobotRun(){
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = std::get<0>(p.goal);
        target_pose.position.y = std::get<1>(p.goal);
        if(target_pose.position.y > 0.3){
             target_pose.position.y = 0.3;
        }
        else if(target_pose.position.y < -0.3){
             target_pose.position.y = -0.3;
        }
        target_pose.position.z = std::get<2>(p.goal);
        if(target_pose.position.z > 0.5){
             target_pose.position.z = 0.5;
        }
        else if(target_pose.position.z < 0.1){
             target_pose.position.z = 0.1;
        }
        if (target_pose.position.y != 0.0 || target_pose.position.z != 0.0) {
            geometry_msgs::msg::Pose current_pose = move_group_.getCurrentPose().pose;
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(current_pose);

            geometry_msgs::msg::Pose eff_target_pose = target_pose;
            eff_target_pose.position.x = current_pose.position.x;
            eff_target_pose.position.y = target_pose.position.y;
            eff_target_pose.position.z = target_pose.position.z;
            eff_target_pose.orientation.w = 1.0;
            RCLCPP_INFO(this->get_logger(), "목표 포즈: x=%f, y=%f, z=%f", eff_target_pose.position.x, eff_target_pose.position.y, eff_target_pose.position.z);
            // Target Pose가 Current Pose와 거의 동일하면 계산하지 않음
            if (fabs(eff_target_pose.position.y-current_pose.position.y) < 0.05 && fabs(eff_target_pose.position.z - current_pose.position.z) < 0.05){
                RCLCPP_INFO(this->get_logger(), "목표 포즈가 현재 포즈와 거의 동일합니다. 계산하지 않습니다.");
                return;
            }
            waypoints.push_back(eff_target_pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 10.0;
            const double eef_step = 0.03;
            double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            RCLCPP_INFO(this->get_logger(), "계획 시각화 (카르테시안 경로) (%.2f%% 달성)", fraction * 100.0);

            // 경로 설정 및 실행
            robot_trajectory::RobotTrajectory rt(move_group_.getCurrentState()->getRobotModel(), PLANNING_GROUP);
            rt.setRobotTrajectoryMsg(*move_group_.getCurrentState(), trajectory);

            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            bool success = iptp.computeTimeStamps(rt, 1.0, 1.0);
            if (!success) {
            RCLCPP_WARN(this->get_logger(), "카르테시안 경로의 시간 매개변수화 실패");
            }

            rt.getRobotTrajectoryMsg(trajectory);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;

            move_group_.execute(plan);

            // 현재 포즈 로그 출력
            geometry_msgs::msg::Pose updated_pose = move_group_.getCurrentPose().pose;
            RCLCPP_INFO(this->get_logger(), "현재 포즈: x=%f, y=%f, z=%f", updated_pose.position.x, updated_pose.position.y, updated_pose.position.z);

            // target pose 초기화
            target_pose.position.x = 0;
            target_pose.position.y = 0;
            target_pose.position.z = 0;
        }
        // 0.5초 대기
        std::this_thread::sleep_for(500ms);
    }

    const std::string PLANNING_GROUP = "moiro_arm";
    moveit::planning_interface::MoveGroupInterface move_group_;
    rclcpp::NodeOptions node_options;
    rclcpp::Client<moiro_interfaces::srv::TargetPose>::SharedPtr srv_client;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto moiro_arm_controller = std::make_shared<MOIROARMMoveGroup>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(moiro_arm_controller);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}