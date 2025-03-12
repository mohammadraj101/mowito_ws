#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "img_conv/srv/img_convert.hpp"
#include <memory>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class ImgService: public rclcpp::Node{

    public:
        ImgService():Node("img_service"), flag(false)
        {
            pub_=this->create_publisher<sensor_msgs::msg::Image>("/processed_img", 10);

            sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/image_raw", 10, std::bind(&ImgService::img_callback,this,std::placeholders::_1));

            service_ = this->create_service<img_conv::srv::ImgConvert>(
                "img_convert",
                std::bind(&ImgService::service_callback, this, std::placeholders::_1,std::placeholders::_2)
            );
            RCLCPP_INFO(this->get_logger(),"Service is Ready: img_convert");
        }
    private:
        bool flag;
        rclcpp::Service<img_conv::srv::ImgConvert>::SharedPtr service_;

        void service_callback(
            const std::shared_ptr<img_conv::srv::ImgConvert::Request>request,
            std::shared_ptr<img_conv::srv::ImgConvert::Response>response)
        {
            if (request->mode){

                flag=true;
                response->result= "Color set to Greyscale";
            }
            else{
                flag=false;
                response->result="Img set to Colour Mode";
            }
            RCLCPP_INFO(this->get_logger(),"Recieved request");

        }

    private:

        void img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            // RCLCPP_INFO(this->get_logger(), "Received an image of size [%u x %u]", msg->width, msg->height);
            try{
                cv::Mat cv_image=cv_bridge::toCvCopy(msg,"bgr8")->image;
                std_msgs::msg::Header header =msg->header;
                sensor_msgs::msg::Image::SharedPtr ros_img;

                if(flag){
                    cv::cvtColor(cv_image,cv_image,cv::COLOR_BGR2GRAY);
                    ros_img = cv_bridge::CvImage(header, "mono8",cv_image).toImageMsg();
                }
                else{
                    ros_img = cv_bridge::CvImage(header, "bgr8",cv_image).toImageMsg();
                }

                cv::imshow("Image",cv_image);
                cv::waitKey(1);


                pub_->publish(*ros_img);
            }
            catch(const cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),"cv_bridge error ; %s", e.what());
            }
        }    
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ImgService>());
    rclcpp::shutdown();
    return 0;
}