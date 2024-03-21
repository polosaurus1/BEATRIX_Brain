#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <serial/serial.h>

class FaceCommandSubscriber : public rclcpp::Node
{
public:
    FaceCommandSubscriber() : Node("face_command_subscriber")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "face_detected", 10, std::bind(&FaceCommandSubscriber::face_detected_callback, this, std::placeholders::_1));

        try {
            my_serial.setPort("/dev/ttyACM0");
            my_serial.setBaudrate(9600);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            my_serial.setTimeout(to);
            my_serial.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port");
        }

        if(my_serial.isOpen()){
            RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Serial Port not open");
        }
    }

private:
    void face_detected_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Face detected, sending command to Arduino.");
            if(my_serial.isOpen()){
                my_serial.write("@MOVRALL 200 200 200 200\r");
            }
        }
    }
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
    serial::Serial my_serial;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceCommandSubscriber>());
    rclcpp::shutdown();
    return 0;
}
