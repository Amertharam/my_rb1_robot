#include "ros/ros.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <asm-generic/errno.h>
#include <cmath>

ros::Publisher vel_pub;
double current_angle = 0.0;
bool first_odom = true;
double initial_angle = 0.0;

void quaternionToEuler(double x, double y, double z, double w, double& roll, double& pitch, double& yaw) {
    // Convert quaternion to Euler angles
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
    double roll, pitch, yaw;
    quaternionToEuler(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, roll, pitch, yaw);

    if (first_odom) {
        initial_angle = yaw;
        first_odom = false;
    }

    current_angle = yaw - initial_angle;
}

bool rotateCallback(my_rb1_ros::Rotate::Request &req, my_rb1_ros::Rotate::Response &res) {
    double target_angle = (req.degrees * M_PI / 180.0) + current_angle;
    geometry_msgs::Twist vel_msg;
    ros::Rate rate(10);
    
    // while (ros::ok() && std::abs(target_angle - current_angle) > 0.01) {
    //     vel_msg.angular.z = 0.1;
    //     vel_pub.publish(vel_msg);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    if (target_angle >= 0) {
        while (ros::ok() && std::abs(target_angle - current_angle) > 0.01) {
        vel_msg.angular.z = 0.1;
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    }
    }
    else {
        while (ros::ok() && std::abs(target_angle - current_angle) > 0.01) {
        vel_msg.angular.z = -0.1;
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    }
    }
    
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
    res.result = "Rotation completed successfully";
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rotate_service");
    ros::NodeHandle n;

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    ros::Subscriber odom_sub = n.subscribe("/odom", 100, odomCallback);
    ros::ServiceServer service = n.advertiseService("/rotate_robot", rotateCallback);

    ros::spin();
    return 0;
}
