#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Nav2GoalClient : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav2GoalClient() : Node("nav2_goal_client")
    {
        // Create the action client
        this->client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // Declare parameters for the goal position and orientation
        this->declare_parameter("x", 15.0); // Default goal position (x)
        this->declare_parameter("y", -4.0); // Default goal position (y)
        this->declare_parameter("z", 0.0); // Default orientation (z)
        this->declare_parameter("w", 1.0); // Default orientation (w)

        // Send the goal after a short delay to ensure the action server is ready
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&Nav2GoalClient::send_goal, this));
    }

    void send_goal()
    {
        // Cancel the timer to ensure the goal is sent only once
        timer_->cancel();

        // Wait for the action server to be available
        if (!this->client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // Create the goal message
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map"; // Ensure the frame_id is correct

        // Get goal parameters
        double x = this->get_parameter("x").as_double();
        double y = this->get_parameter("y").as_double();
        double z = this->get_parameter("z").as_double();
        double w = this->get_parameter("w").as_double();

        // Set the goal position and orientation
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.z = z;
        goal_msg.pose.pose.orientation.w = w;

        RCLCPP_INFO(this->get_logger(), "Sending goal: x=%f, y=%f, z=%f, w=%f", x, y, z, w);

        // Set up goal options
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&Nav2GoalClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&Nav2GoalClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&Nav2GoalClient::result_callback, this, std::placeholders::_1);

        // Send the goal
        this->client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %f", feedback->distance_remaining);
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2GoalClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}