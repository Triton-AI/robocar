#ifndef TWIST_TO_ACKERMANN__TWIST_TO_ACKERMANN_NODE_HPP_
#define TWIST_TO_ACKERMANN__TWIST_TO_ACKERMANN_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


class TwistToAckermann : public rclcpp::Node
{
public:
    /**
     * TwistToAckermann constructor. This constructor creates a ros2 node. 
     */
    TwistToAckermann();

    /**
     * TwistToAckermann destructor. This destructor destroys a tta node. 
     */
    ~TwistToAckermann();

    /**
     * Finds the appropriate steering angle for some turning radius, which is defined by the 
     * transient velocity of the vehicle along that curve (just the current velocity in this instance), 
     * and the rate that curve changes (angular z rotation). This angle is effected by vehicle wheelbase.
     * 
     * All units are in m/s or rad/s.
     */
    float convert_trans_rot_vel_to_steering_angle(float vel, float omega, float wheelbase);



private:
    // Publishers and Subscribers
    // Without stamps
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr _ack_pub = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_sub = nullptr;
    // With stamps
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _ack_S_pub = nullptr;

    /// Define timer_callback, things the node will execute periodically. 
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer; 
    rclcpp::Time current_time;

    /// Define twist_callback, subscribe periodically. 
    void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg);

    /// Wheelbase in meters.
    float _wheelbase{};

    /// Maximum Steering Angle in degrees
    float max_steering_angle_deg;
    /// Maximum Steering Angle in radians
    float max_steering_angle;
    /// default_steering_direction: counter-clockwise positive: 0, clockwise positive: 1
    int default_steering_direction;

    /// Whether to use stamped messages.
    bool _use_stamps;

    std::string ack_topic;
    std::string twist_topic;
    std::string frame_id;
};

#endif  // TWIST_TO_ACKERMANN__TWIST_TO_ACKERMANN_NODE_HPP_