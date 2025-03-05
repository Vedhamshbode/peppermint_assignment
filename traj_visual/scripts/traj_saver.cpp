#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include "custom_service/srv/traj_sw.hpp"


using namespace std::chrono_literals;
using namespace std;
using Matrix4x4 = std::array<std::array<double, 4>, 4>;
using Vector3 = std::array<double, 3>;
using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

class TrajSaver : public rclcpp::Node
{
public:
    TrajSaver() : Node("traj_saver")
    {


        service_= this->create_service<custom_service::srv::TrajSw>("traj_service", bind(&TrajSaver::service_callback,this, placeholders::_1, placeholders::_2));
        // RCLCPP_INFO(this->get_logger(), "Traj_ServiceNode has been started.");

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",    // Topic name
            10,         // Queue size
            std::bind(&TrajSaver::odom_callback, this, std::placeholders::_1)
        );
        req= false, res= false;
        void save_to_csv(const std::string& file_name);
        // RCLCPP_INFO(this->get_logger(), "Odometry Subscriber Node Started");   
     }

     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received odometry update.");
    
    try {
        std::array<double, 3> point = {
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        };

        trajectory_points_.push_back(point);
        marker_pub(trajectory_points_);
        
        if (req) {
            rclcpp::Duration elapsed_time = this->now() - start_time_;
            points.push_back({msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z});

            if (elapsed_time.seconds() >= record_time_sec_) {
                save_to_csv(name);
                RCLCPP_INFO(this->get_logger(), "Recording stopped after %.2f seconds.", record_time_sec_);
            }
        }
    }
     catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in odom_callback: %s", e.what());
    }
        
    }

    void save_to_csv(const std::string& file_name) {
        std::ofstream file(file_name);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing!");
            return;
        }

        file << "X,Y,Z\n";

        for (const auto &row : points) {
            file << row[0] << "," << row[1] << "," << row[2] << "\n";
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Odometry data saved to odom_data.csv!");
        res = true;
    }

    void service_callback(const custom_service::srv::TrajSw_Request::SharedPtr request, const custom_service::srv::TrajSw_Response::SharedPtr response)
    {

        RCLCPP_INFO(this->get_logger(), "request received!");

        name = request->filename;
        float duration = request->duration;
        req = true;
        record_time_sec_ = duration;
        start_time_ = this->now();
        if (res){
            response->success= true;
        }
    }

    void marker_pub(std::vector<std::array<double, 3>> pts ){
        cout<<"func called"<<endl;
        // ******************* marker code ******************
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker marker;
        for (int i =0; i<pts.size(); i++){
            marker.header.frame_id = "odom";  // Change to "odom" or "base_link" if needed
            marker.header.stamp = this->now();
            marker.ns = "marker_points";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
    
            marker.pose.position.x = pts[i][0];
            marker.pose.position.y = pts[i][1];
            marker.pose.position.z = pts[i][2];
    
            marker.scale.x = 0.05;  // Marker size
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
    
            marker.color.a = 1.0;  // Fully visible
            marker.color.r = 1.0;  // Red color
            marker.color.g = 0.0;
            marker.color.b = 0.0;
    
            marker.lifetime = rclcpp::Duration::from_seconds(0);  // Infinite lifetime
    
            marker_array.markers.push_back(marker);
            publisher_->publish(marker_array);
        }
    }
    




    private:

    std::ofstream file_;
    std::vector<std::array<double, 3>> trajectory_points_;
    std::vector<std::vector<double>> points;

    std::vector<std::thread> threads_;
    bool req,res;
    std::string name;
    rclcpp::Time start_time_;
    float record_time_sec_ = 0.0;
    double i = 0;
    rclcpp::Service<custom_service::srv::TrajSw>::SharedPtr service_;

    rclcpp::Client<custom_service::srv::TrajSw>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<TrajSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
