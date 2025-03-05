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
#include <sstream>



using namespace std::chrono_literals;
using namespace std;
using Matrix4x4 = std::array<std::array<double, 4>, 4>;
using Vector3 = std::array<double, 3>;
using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

class TrajRead : public rclcpp::Node
{
public:
    TrajRead() : Node("traj_read")
    {
        this->declare_parameter<std::string>("filename", "/home/vedh/workspaces/pep_ws/trajectory.csv");
        filename = this->get_parameter("filename").as_string();
        RCLCPP_INFO(this->get_logger(), "Filename: %s", filename.c_str());
      

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);

        marker_pub();
    }

    std::vector<std::vector<double>> read_csv(const std::string &filename) {
        std::vector<std::vector<double>> data;
        std::ifstream file(filename);

        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filename.c_str());
            return data;
        }

        std::string line;

    if (std::getline(file, line)) {
        RCLCPP_INFO(this->get_logger(), "Skipping header: %s", line.c_str());
    }
        while (std::getline(file, line)) {
            std::vector<double> row;
            std::stringstream ss(line);
            std::string value;

            while (std::getline(ss, value, ',')) {
                row.push_back(std::stod(value));  // Convert string to double
            }

            if (!row.empty()) {
                data.push_back(row);
                cout<<data[0][0];
            }
        }

        file.close();
        return data;
    }

    

    void marker_pub(){
        std::vector<std::vector<double>> data = read_csv(filename);
        // ******************* marker code ******************
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker marker;
        for (int i =0; i<data.size(); i++){
            marker.header.frame_id = "odom";  // Change to "odom" or "base_link" if needed
            marker.header.stamp = this->now();
            marker.ns = "marker_points";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
    
            marker.pose.position.x = data[i][0];
            marker.pose.position.y = data[i][1];
            marker.pose.position.z = data[i][2];
    
            marker.scale.x = 0.01;  // Marker size
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
    
            marker.color.a = 1.0;  // Fully visible
            marker.color.r = 1.0;  // Red color
            marker.color.g = 0.0;
            marker.color.b = 0.0;
    
            marker.lifetime = rclcpp::Duration::from_seconds(0);  // Infinite lifetime
    
            marker_array.markers.push_back(marker);
            publisher_->publish(marker_array);
            cout<<"pub marker"<<endl;
        }
    }
private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    string filename;


};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<TrajRead>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
