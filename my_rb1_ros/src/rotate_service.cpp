#include "ros/ros.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

class RotateService {
private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber odom_sub_;
    nav_msgs::Odometry::ConstPtr last_odom_msg_;
    const double FIXED_ANGULAR_VELOCITY = 0.4;
    const double ANGLE_TOLERANCE = 0.01745;
    const double Kp = 1.0;

public:
    RotateService() : nh_("~") {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
        odom_sub_ = nh_.subscribe("/odom", 100, &RotateService::odomCallback, this);
    }

    void quaternionToEuler(double x, double y, double z, double w, double& roll, double& pitch, double& yaw) {
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);
        else
            pitch = std::asin(sinp);

        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        last_odom_msg_ = msg;
    }

    bool rotateCallback(my_rb1_ros::Rotate::Request &req, my_rb1_ros::Rotate::Response &res) {
        ROS_INFO("Service Requested: Rotate Robot");

        double target_angle_degrees = req.degrees;
        double target_angle_radians = target_angle_degrees * M_PI / 180.0;

        geometry_msgs::Twist vel_msg;
        ros::Rate rate(50);

        ros::spinOnce();
        if (!last_odom_msg_) {
            ROS_ERROR("No odometry message received. Aborting rotation.");
            res.result = "Error: No odometry message received.";
            return false;
        }

        double roll, pitch, current_yaw;
        quaternionToEuler(last_odom_msg_->pose.pose.orientation.x, last_odom_msg_->pose.pose.orientation.y, last_odom_msg_->pose.pose.orientation.z, last_odom_msg_->pose.pose.orientation.w, roll, pitch, current_yaw);

        double target_yaw; 

        if (std::abs(target_angle_degrees) == 360.0) {
            target_angle_radians = (target_angle_degrees > 0) ? 2 * M_PI : -2 * M_PI;
            target_yaw = current_yaw + target_angle_radians;
        } else {
            target_yaw = current_yaw + target_angle_radians;
            target_yaw = std::atan2(std::sin(target_yaw), std::cos(target_yaw));
        }

        double angular_speed = (target_angle_degrees >= 0) ? FIXED_ANGULAR_VELOCITY : -FIXED_ANGULAR_VELOCITY;
        double initial_yaw = current_yaw;
        double total_rotated = 0.0; // Track total rotation

        while (ros::ok()) {
            ros::spinOnce();

            if (!last_odom_msg_) {
                ROS_WARN_ONCE("Waiting for odometry message...");
                continue;
            }

            quaternionToEuler(last_odom_msg_->pose.pose.orientation.x, last_odom_msg_->pose.pose.orientation.y, last_odom_msg_->pose.pose.orientation.z, last_odom_msg_->pose.pose.orientation.w, roll, pitch, current_yaw);

            double error = target_yaw - current_yaw;
            error = std::atan2(std::sin(error), std::cos(error));

            if (std::abs(target_angle_degrees) == 360.0) {
                double delta_yaw = current_yaw - initial_yaw;
                delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw)); // Normalize delta

                total_rotated += delta_yaw;
                if (std::abs(total_rotated) >= 2 * M_PI - ANGLE_TOLERANCE) {
                    break;
                }
                initial_yaw = current_yaw; //Update initial yaw after each iteration.
            } else if (std::abs(error) <= ANGLE_TOLERANCE) {
                break;
            }

            double speed_multiplier = std::max(0.75, std::min(10.0, Kp * std::abs(error)));

            vel_msg.angular.z = angular_speed * speed_multiplier;
            vel_pub_.publish(vel_msg);
            rate.sleep();
        }

        vel_msg.angular.z = 0.0;
        vel_pub_.publish(vel_msg);

        ROS_INFO("Service Completed: Rotation completed successfully");
        res.result = "Rotation completed successfully";
        return true;
    }
    ros::ServiceServer service = nh_.advertiseService("/rotate_robot", &RotateService::rotateCallback, this);
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rotate_service");
    RotateService rotate_service;
    ROS_INFO("Service Ready: Rotate Robot Service is ready to receive requests");
    ros::spin();
    return 0;
}