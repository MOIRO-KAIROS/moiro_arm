#include <rclcpp/rclcpp.hpp>  // ROS2 Library Include
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>

#include <yolov8_msgs/srv/target_pose.hpp>  // Custom Service Include

static const double PLANNING_TIME_S = 10.0; // Planning Time 설정


class MycobotMoveGroup : public rclcpp::Node
{
public:
    MycobotMoveGroup(rclcpp::NodeOptions& options) : Node("mycobot_move_group", options)
    {
        node_handle_ = rclcpp::Node::make_shared("mycobot_move_group_interface", options)
        
        // MoveIt2 Move Group Interface 생성 및 설정
        move_group_ = new moveit::planning_interface::MoveGroupInterface(node_handle_, "mycobot_arm");
        move_group_->setPlanningTime(PLANNING_TIME_S); // Planning Time 설정
        move_group_->startStateMonitor(10);
         RCLCPP_INFO(this->get_logger(), "Move Group Interface Created");
        joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup("mycobot_arm");
        RCLCPP_INFO(this->get_logger(), "Planning frame : %s", move_group_->getPlanningFrame().c_str());
        // joint model group check
        RCLCPP_INFO(this->get_logger(), "Joint Model Group: %s", joint_model_group_->getName().c_str());
        RCLCPP_INFO(this->get_logger(), "Joint Model Group Variable Count: %d", joint_model_group_->getVariableCount());
        const std::vector<std::string>& joint_names = joint_model_group_->getVariableNames();
        for (const auto& joint_name : joint_names)
        {
            RCLCPP_INFO(this->get_logger(), "Joint Name: %s", joint_name.c_str());
        }
        // end effector link check
        RCLCPP_INFO(this->get_logger(), "End Effector Link: %s", move_group_->getEndEffectorLink().c_str());
        // Current Pose Debug
        move_group_->setStartState(*move_group_->getCurrentState()); // 

        RCLCPP_INFO(this->get_logger(), "Current Pose : x=%f, y=%f, z=%f", move_group_->getCurrentPose().pose.position.x, move_group_->getCurrentPose().pose.position.y, move_group_->getCurrentPose().pose.position.z);
        
        // QoS Profile 설정
        rmw_qos_profile_t custom_qos_profile 
        {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            10,
            RMW_QOS_POLICY_RELIABILITY_RELIABLE
        };

        // Service Server 생성
        target_pose_service_ = this->create_service<yolov8_msgs::srv::TargetPose>(
            "yolo/target_pose", std::bind(&MycobotMoveGroup::targetPoseCallback, this, std::placeholders::_1, std::placeholders::_2), custom_qos_profile);
    }

    void targetPoseCallback(
        const std::shared_ptr<yolov8_msgs::srv::TargetPose::Request> request,
        std::shared_ptr<yolov8_msgs::srv::TargetPose::Response> response)
    {
        // Current Pose Debug
        move_group_->setStartState(*move_group_->getCurrentState());
        geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose("link6");
        RCLCPP_INFO(this->get_logger(), "Current Pose: x=%f, y=%f, z=%f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

        // MoveIt2 Move Group Interface를 이용하여 Target Pose 설정
        geometry_msgs::msg::Pose target_pose_z;
        target_pose_z.position.x = current_pose.pose.position.x;
        target_pose_z.position.y = current_pose.pose.position.y;
        target_pose_z.position.z = request->z;
        target_pose_z.orientation.w = 1.0;

        move_group_->setPoseTarget(target_pose_z);

        // Path Constraints 설정
        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = "gripper_base";
        ocm.header.frame_id = "base_link";
        ocm.orientation.w = 1.0;
        ocm.absolute_x_axis_tolerance = 0.5;
        ocm.absolute_y_axis_tolerance = 0.5;
        ocm.absolute_z_axis_tolerance = 0.5;
        ocm.weight = 1.0;
        moveit_msgs::msg::Constraints constraints;
        constraints.orientation_constraints.push_back(ocm);
        move_group_->setPathConstraints(constraints);

        // Target Pose로 이동 계획 생성
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // 결과 Debug 및 반환
        RCLCPP_INFO(this->get_logger(), "Plan %s", success ? "Succeeded" : "Failed");
        response->success = success;

        // Target Pose로 이동 실행
        if (success)
        {
            move_group_->asyncExecute(plan);
            rclcpp::sleep_for(std::chrono::seconds(1)); 
            move_group_->setStartStateToCurrentState();
            current_pose = move_group_->getCurrentPose("link6");
            RCLCPP_INFO(this->get_logger(), "After Move, Current Pose: x=%f, y=%f, z=%f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
            return;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to Plan");
            return;
        }
    }
private:

    rclcpp::Node::SharedPtr node_handle_;
    moveit::planning_interface::MoveGroupInterface* move_group_;
    const moveit::core::JointModelGroup* joint_model_group_;
    rclcpp::Service<yolov8_msgs::srv::TargetPose>::SharedPtr target_pose_service_;
    moveit::core::RobotStatePtr current_state_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto mycobot_move_group = std::make_shared<MycobotMoveGroup>(options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(mycobot_move_group);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
