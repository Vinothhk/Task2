#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>

class DifferentialDriveController : public rclcpp::Node {
public:
    DifferentialDriveController() : Node("differential_drive_controller") {
        // Declare parameters with default values
        this->declare_parameter("wheelbase", 0.5);
        this->declare_parameter("wheel_radius", 0.1);
        this->declare_parameter("max_rpm", 100.0);
        
        // Initialize parameters
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_rpm_ = this->get_parameter("max_rpm").as_double();

        // Set up a callback for parameter changes
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DifferentialDriveController::onParameterChange, this, std::placeholders::_1)
        );

        // Subscriber to cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));

        // Publishers for left and right wheel RPM (now using Float32)
        left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float32>("/left_wheel_rpm", 10);
        right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float32>("/right_wheel_rpm", 10);
    }

private:
    // Callback function to handle parameter changes
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &params) {
        for (const auto &param : params) {
            if (param.get_name() == "wheelbase") {
                wheelbase_ = param.as_double();
            } else if (param.get_name() == "wheel_radius") {
                wheel_radius_ = param.as_double();
            } else if (param.get_name() == "max_rpm") {
                max_rpm_ = param.as_double();
            }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear_vel = msg->linear.x;
        double angular_vel = msg->angular.z;

        // Compute left and right wheel velocities
        double v_left = linear_vel - (angular_vel * wheelbase_ / 2.0);
        double v_right = linear_vel + (angular_vel * wheelbase_ / 2.0);

        // Convert to RPM
        double left_rpm = (v_left / (2.0 * M_PI * wheel_radius_)) * 60.0;
        double right_rpm = (v_right / (2.0 * M_PI * wheel_radius_)) * 60.0;

        // Limit RPM to max_rpm_
        left_rpm = std::clamp(left_rpm, -max_rpm_, max_rpm_);
        right_rpm = std::clamp(right_rpm, -max_rpm_, max_rpm_);

        // Publish RPM values (now using Float32)
        std_msgs::msg::Float32 left_msg, right_msg;
        left_msg.data = static_cast<float>(left_rpm);
        right_msg.data = static_cast<float>(right_rpm);

        left_wheel_pub_->publish(left_msg);
        right_wheel_pub_->publish(right_msg);
    }

    double wheelbase_, wheel_radius_, max_rpm_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_wheel_pub_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialDriveController>());
    rclcpp::shutdown();
    return 0;
}
