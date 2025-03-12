#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
// #include "img_conv/srv/img_convert.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class ImgSubscriber: public rclcpp::Node{
    public:
        ImgSubscriber(): Node("img_subscriber")
        {
            sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/processed_img", 10, std::bind(&ImgSubscriber::img_callback,this,std::placeholders::_1));

        }
    private:
        void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
        {
            // RCLCPP_INFO(this->get_logger(), "Received an image of size [%u x %u]", msg->width, msg->height);
            try{
                cv::Mat cv_image=cv_bridge::toCvCopy(msg,msg->encoding)->image;
                cv::imshow("Processed Image",cv_image);
                cv::waitKey(1);
            }
            catch(const cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),"cv_bridge error ; %s", e.what());
            }
        }    
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImgSubscriber>());
    rclcpp::shutdown();
    return 0;
}

