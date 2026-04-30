#include "../common/hik_camera.h"

using std::placeholders::_1;
using namespace std;

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("hik_camera_image_subscriber"),nDepthSeq(0),nRgbSeq(0),
    nLeftIrSeq(0),nRightIrSeq(0)
    {
        mkdir("src/mv3d_rgbd_ros2/image", S_IRWXU);
        strSavePath = "src/mv3d_rgbd_ros2/image";
        RCLCPP_INFO(this->get_logger(), "[Subscriber] save path: %s", strSavePath.c_str());
        depthSub = this->create_subscription<sensor_msgs::msg::Image>("/camera/depth", 10, std::bind(&ImageSubscriber::depthCallback, this, _1));
        rgbSub = this->create_subscription<sensor_msgs::msg::Image>("/camera/rgb", 10, std::bind(&ImageSubscriber::rgbCallback, this, _1));
        leftIrSub = this->create_subscription<sensor_msgs::msg::Image>("/camera/leftIr", 10, std::bind(&ImageSubscriber::leftIrCallback, this, _1));
        rightIrSub = this->create_subscription<sensor_msgs::msg::Image>("/camera/rightIr", 10, std::bind(&ImageSubscriber::rightIrCallback, this, _1));
    }

private:
    uint32_t nDepthSeq;
    uint32_t nRgbSeq;
    uint32_t nLeftIrSeq;
    uint32_t nRightIrSeq;
    string strSavePath;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgbSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr leftIrSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rightIrSub;
    const void depthCallback(const sensor_msgs::msg::Image::Ptr msg)
    {
        RCLCPP_INFO(this->get_logger(), "[Subscriber] Get depth image[%d]", nDepthSeq++);
        cv_bridge::CvImagePtr depthPtr;
        try
        {
            depthPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[Subscriber] cv_bridge depth image Error! %s", e.what());
            depthPtr = nullptr;
        }
        if(nullptr != depthPtr)
        {
            // 保存深度图
            char chFileName[256] = "";
            sprintf(chFileName, "%s/[%d]_DepthImage.png", strSavePath.c_str(), nDepthSeq - 1);
            cv::imwrite(chFileName, depthPtr->image);
            RCLCPP_INFO(this->get_logger(), "[Subscriber] Save depth image [%d] success", nDepthSeq - 1);
        }
    }
    const void rgbCallback(const sensor_msgs::msg::Image::Ptr msg)
    {
        RCLCPP_INFO(this->get_logger(), "[Subscriber] Get rgb image[%d]", nRgbSeq++);
        cv_bridge::CvImagePtr RgbPtr;
        try
        {
            RgbPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[Subscriber] cv_bridge Rgb image Error! %s", e.what());
            RgbPtr = nullptr;
        }
        if(nullptr != RgbPtr)
        {
            // 保存彩色图
            char chFileName[256] = "";
            sprintf(chFileName, "%s/[%d]_RgbImage.png", strSavePath.c_str(), nRgbSeq - 1);
            cv::imwrite(chFileName, RgbPtr->image);
            RCLCPP_INFO(this->get_logger(), "[Subscriber] Save rgb image [%d] success", nRgbSeq - 1);
        }
    }
    const void leftIrCallback(const sensor_msgs::msg::Image::Ptr msg)
    {
        RCLCPP_INFO(this->get_logger(), "[Subscriber] Get left ir image[%d]", nLeftIrSeq++);
        cv_bridge::CvImagePtr leftIrPtr;
        try
        {
            leftIrPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[Subscriber] cv_bridge left ir image Error! %s", e.what());
            leftIrPtr = nullptr;
        }
        if(nullptr != leftIrPtr)
        {
            // 保存左目原始图
            char chFileName[256] = "";
            sprintf(chFileName, "%s/[%d]_LeftIrImage.png", strSavePath.c_str(), nLeftIrSeq - 1);
            cv::imwrite(chFileName, leftIrPtr->image);
            RCLCPP_INFO(this->get_logger(), "[Subscriber] Save left ir image [%d] success", nLeftIrSeq - 1);
        }
    }
    const void rightIrCallback(const sensor_msgs::msg::Image::Ptr msg)
    {
        RCLCPP_INFO(this->get_logger(), "[Subscriber] Get right ir image[%d]", nRightIrSeq++);
        cv_bridge::CvImagePtr rightIrPtr;
        try
        {
            rightIrPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[Subscriber] cv_bridge rightIr image Error! %s", e.what());
            rightIrPtr = nullptr;
        }
        if(nullptr != rightIrPtr)
        {
            // 保存右目原始图
            char chFileName[256] = "";
            sprintf(chFileName, "%s/[%d]_RightIrImage.png", strSavePath.c_str(), nRightIrSeq - 1);
            cv::imwrite(chFileName, rightIrPtr->image);
            RCLCPP_INFO(this->get_logger(), "[Subscriber] Save right ir image [%d] success", nRightIrSeq - 1);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
