#include "../common/hik_camera.h"
#include "mv3d_rgbd_ros2_interface/srv/image.hpp"

using mv3d_rgbd_ros2_interface::srv::Image;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

class ImageServer : public rclcpp::Node
{
public:
    ImageServer():Node("hik_camera_image_server")
    {
        RCLCPP_INFO(this->get_logger(), "-------------------- Server start --------------------");
        mkdir("src/mv3d_rgbd_ros2/image", S_IRWXU);
        strSavePath = "src/mv3d_rgbd_ros2/image";
        RCLCPP_INFO(this->get_logger(), "[Subscriber] save path: %s", strSavePath.c_str());
        server = this->create_service<Image>("SaveImage", std::bind(&ImageServer::saveImage, this, _1, _2));
    }
private:
    rclcpp::Service<Image>::SharedPtr server;
    string strSavePath;
    void saveImage(const Image::Request::SharedPtr request, const Image::Response::SharedPtr response)
    {
        // 获取帧号
        uint32_t nFrameNum = request->frame_num;
        // 解析图像类型
        uint32_t nMsgImgType = request->img_type;
        // 获取相机序列号
        string strSerialNum = request->cam_sn;
        if(nMsgImgType & TYPE_DEPTH)
        {
            RCLCPP_INFO(this->get_logger(), "[Server] SN:%s Get depth image[%d] success", strSerialNum.c_str(), nFrameNum);
            cv_bridge::CvImagePtr depthPtr;
            try
            {
                depthPtr = cv_bridge::toCvCopy(request->depth_img, sensor_msgs::image_encodings::TYPE_16UC1);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "[Server] SN:%s cv_bridge depth image Error! %s", strSerialNum.c_str(), e.what());
                depthPtr = nullptr;
            }
            if(nullptr != depthPtr)
            {
                // 保存深度图
                char chFileName[256] = "";
                sprintf(chFileName, "%s/[%d]_DepthImage.png", strSavePath.c_str(), nFrameNum);
                cv::imwrite(chFileName, depthPtr->image);
                RCLCPP_INFO(this->get_logger(), "[Server] SN:%s Save [%d]_DepthImage.png success", strSerialNum.c_str(), nFrameNum);
            }
        }
        if(nMsgImgType & TYPE_RGB)
        {
            RCLCPP_INFO(this->get_logger(), "[Server] SN:%s Get rgb image[%d] success", strSerialNum.c_str(), nFrameNum);
            cv_bridge::CvImagePtr rgbPtr;
            try
            {
                rgbPtr = cv_bridge::toCvCopy(request->rgb_img, sensor_msgs::image_encodings::BGR8);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "[Server] SN:%s cv_bridge rgb image Error! %s", strSerialNum.c_str(), e.what());
                rgbPtr = nullptr;
            }
            if(nullptr != rgbPtr)
            {
                // 保存彩色图
                char chFileName[256] = "";
                sprintf(chFileName, "%s/[%d]_RgbImage.png", strSavePath.c_str(), nFrameNum);
                cv::imwrite(chFileName, rgbPtr->image);
                RCLCPP_INFO(this->get_logger(), "[Server] SN:%s Save [%d]_RgbImage.png success", strSerialNum.c_str(), nFrameNum);
            }
        }
        if(nMsgImgType & TYPE_LEFT_IR)
        {
            RCLCPP_INFO(this->get_logger(), "[Server] SN:%s Get left ir image[%d] success", strSerialNum.c_str(), nFrameNum);
            cv_bridge::CvImagePtr leftIrPtr;
            try
            {
                leftIrPtr = cv_bridge::toCvCopy(request->left_ir_img, sensor_msgs::image_encodings::TYPE_8UC1);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "[Server] SN:%s cv_bridge leftIr image Error! %s", strSerialNum.c_str(), e.what());
                leftIrPtr = nullptr;
            }
            if(nullptr != leftIrPtr)
            {
                // 保存左目IR图
                char chFileName[256] = "";
                sprintf(chFileName, "%s/[%d]_LeftIrImage.png", strSavePath.c_str(), nFrameNum);
                cv::imwrite(chFileName, leftIrPtr->image);
                RCLCPP_INFO(this->get_logger(), "[Server] SN:%s Save [%d]_LeftIrImage.png success", strSerialNum.c_str(), nFrameNum);
            }
        }
        if(nMsgImgType & TYPE_RIGHT_IR)
        {
            RCLCPP_INFO(this->get_logger(), "[Server] SN:%s Get right ir image[%d] success", strSerialNum.c_str(), nFrameNum);
            cv_bridge::CvImagePtr rightIrPtr;
            try
            {
                rightIrPtr = cv_bridge::toCvCopy(request->right_ir_img, sensor_msgs::image_encodings::TYPE_8UC1);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "[Server] SN:%s cv_bridge rightIr image Error! %s", strSerialNum.c_str(), e.what());
                rightIrPtr = nullptr;
            }
            if(nullptr != rightIrPtr)
            {
                // 保存右目IR图
                char chFileName[256] = "";
                sprintf(chFileName, "%s/[%d]_RightIrImage.png", strSavePath.c_str(), nFrameNum);
                cv::imwrite(chFileName, rightIrPtr->image);
                RCLCPP_INFO(this->get_logger(), "[Server] SN:%s Save [%d]_RightIrImage.png success", strSerialNum.c_str(), nFrameNum);
            }
        }
    }

};

int main(int argc, char *argv[])
{
    // 初始化ros节点
    rclcpp::init(argc, argv);
    auto server = std::make_shared<ImageServer>();
    // 调用spin函数，并传入节点对象指针
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
}
