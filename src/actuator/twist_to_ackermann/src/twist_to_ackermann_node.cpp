#include "twist_to_ackermann/twist_to_ackermann_node.hpp"

template <typename T> float sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

TwistToAckermann::TwistToAckermann()
    : Node("twist_to_ackermann_node")
{
    _use_stamps = this->declare_parameter("use_stamps_on_ackermann", false);

    if (_use_stamps)
        RCLCPP_INFO(this->get_logger(), "Start converting twist to ackermann_drive_stamped");
    else
        RCLCPP_INFO(this->get_logger(), "Start converting twist to ackermann_drive");

    ack_topic = this->declare_parameter("ackermann_topic", "/ack_cmd");
    twist_topic = this->declare_parameter("twist_topic", "/twist_cmd");

    frame_id = this->declare_parameter("frame_id", "odom");

    _wheelbase = this->declare_parameter("wheelbase", 1.0);
    max_steering_angle_deg = this->declare_parameter("max_steering_angle_deg", 70.0);

    if (_use_stamps)
    {
        _ack_S_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ack_topic, 10);
    }
    else
    {
        _ack_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(ack_topic, 10);
    }
    _twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(twist_topic, 
                                                                      10, 
                                                                      std::bind(&TwistToAckermann::twist_callback, this, std::placeholders::_1));

    current_time = this->get_clock()->now();
    timer = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&TwistToAckermann::timer_callback, this)
    );
}

TwistToAckermann::~TwistToAckermann()
{
    RCLCPP_INFO(this->get_logger(), "Destorying twist_to_ackermann Node");
}

float TwistToAckermann::convert_trans_rot_vel_to_steering_angle(float vel, float omega, float wheelbase)
{
    if (omega == 0 || vel == 0)
    {
        return 0.0;
    }
    
    // Remove negative so steering doesn't reverse when reversing.
    vel = std::abs(vel);

    auto radius = vel / omega;
    float desired_steering_angle = std::atan(wheelbase / radius);
    if (desired_steering_angle > -max_steering_angle_deg || desired_steering_angle < max_steering_angle_deg)
    {
        desired_steering_angle = sgn(desired_steering_angle) * max_steering_angle_deg;
    }

    return desired_steering_angle;
}

void TwistToAckermann::twist_callback(geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (_use_stamps)
    {
        ackermann_msgs::msg::AckermannDriveStamped out{};
        out.header.frame_id = this->frame_id;
        out.header.stamp = this->current_time;

        out.drive.speed = msg->linear.x;
        out.drive.steering_angle = convert_trans_rot_vel_to_steering_angle(msg->linear.x,
                                                                           msg->angular.z, 
                                                                           this->_wheelbase);
        _ack_S_pub->publish(out);
    }
    else
    {
        ackermann_msgs::msg::AckermannDrive out{};
        out.speed = msg->linear.x;
        out.steering_angle = convert_trans_rot_vel_to_steering_angle(msg->linear.x,
                                                                     msg->angular.z, 
                                                                     this->_wheelbase);
        _ack_pub->publish(out);
    }
}

void TwistToAckermann::timer_callback()
{
    // RCLCPP_INFO(this->get_logger(), "timer working");
    current_time = this->get_clock()->now();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<TwistToAckermann>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();

    return 0;
}
