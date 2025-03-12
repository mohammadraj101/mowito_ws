#include "rclcpp/rclcpp.hpp"
#include "img_conv/srv/img_convert.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>

using namespace std::chrono_literals;

class ImgClient : public rclcpp::Node{

    public: 
        ImgClient(): Node("img_client")
        {
            cli_=this->create_client<img_conv::srv::ImgConvert>("/img_convert");

            while(!cli_->wait_for_service(1s)){
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(),"Interuped in waiting for service");
                    return;
                }
                RCLCPP_INFO(this->get_logger(),"Waiting for service to be available.....");
            }

            auto request_ = std::make_shared<img_conv::srv::ImgConvert::Request>();
            request_->mode = true;

            auto result =cli_->async_send_request(request_);

            if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)==rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "Result: %s", result.get()->result.c_str());
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Failed to call service");
            }
        }

    private:
        rclcpp::Client<img_conv::srv::ImgConvert>::SharedPtr cli_;
        // void client_callback()
        // {
            
        // }

};

int main(int args, char **argv){
    rclcpp::init(args,argv);
    rclcpp::spin(std::make_shared<ImgClient>());
    rclcpp::shutdown();
    return(0);
}