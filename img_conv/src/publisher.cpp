#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;

class CPPPublisher : public rclcpp::Node{
    public:
        CPPPublisher() : Node("cpp_publisher"), count_(0){
            publisher_=this->create_publisher<std_msgs::msg::String>("/chatter", 10);
            timer_=this->create_wall_timer(500ms, std::bind(&CPPPublisher::publish_message, this));
        }

    private:
        void publish_message() {
            auto message =std_msgs::msg::String();
            message.data ="Hi thi is Raj count: " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(),"Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<CPPPublisher>());
    rclcpp::shutdown();
    return 0;
}