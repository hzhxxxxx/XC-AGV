#include "../common/hik_camera.h"
#include "mv3d_rgbd_ros2_interface/srv/image.hpp"

using mv3d_rgbd_ros2_interface::srv::Image;
using namespace std;
using namespace std::chrono_literals;

// 定义节点类
class ImageClient: public rclcpp::Node{
public:
    ImageClient():Node("hik_camera_image_client")
    {
        this->declare_parameter<std::string>("cam_sn", "None");
        RCLCPP_INFO(this->get_logger(),"-------------------- Client start --------------------");
        // 初始化相机并连接
        camera.initialize();
        string strSerialNumber = "None";
        // 获取序列号
        this->get_parameter("cam_sn", strSerialNumber);
        if(strSerialNumber == "None")
        {
            // 默认连接第一个相机
            camera.connect();
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("hik_camera_image_client"), "---------------- Connect by serial number: %s ----------------", strSerialNumber.c_str());
            // 根据序列号进行连接
            camera.connect(strSerialNumber.c_str());
        }
        // 获取标定信息
        m_stCamInfo = camera.getCamInfo();
        // 获取相机序列号
        m_strCamSerialNumber = camera.getSerialNumber();
        // 创建客户端
        client = this->create_client<Image>("SaveImage");
    }
    // 等待服务连接
    bool connect_server()
    {
        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"[Client] SN:%s Connect to server...", m_strCamSerialNumber.c_str());
        }
        return true;
    }
    // 组织请求数据并发送
    rclcpp::Client<Image>::FutureAndRequestId send_request(vector<sensor_msgs::msg::Image::Ptr>& stImagePtr,
    int64_t nTimeStamp, uint32_t nImgType, uint32_t nFrameNum)
    {
        auto request = std::make_shared<Image::Request>();
        request->cam_sn = m_strCamSerialNumber;
        request->time_stamp = nTimeStamp;
        request->img_type = nImgType;
        request->frame_num = nFrameNum;
        request->depth_cam_info = m_stCamInfo[0];
        request->rgb_cam_info = m_stCamInfo[1];
        if(nImgType & TYPE_DEPTH)
        {
            request->depth_img = *stImagePtr[0];
        }
        if(nImgType & TYPE_RGB)
        {
            request->rgb_img = *stImagePtr[1];
        }
        if(nImgType & TYPE_LEFT_IR)
        {
            request->left_ir_img = *stImagePtr[2];
        }
        if(nImgType & TYPE_RIGHT_IR)
        {
            request->right_ir_img = *stImagePtr[3];
        }
        return client->async_send_request(request);
    }

    HikCamera camera;
    string m_strCamSerialNumber;
private:
    rclcpp::Client<Image>::SharedPtr client;
    rclcpp::TimerBase::SharedPtr timer;
    vector<sensor_msgs::msg::CameraInfo> m_stCamInfo;
};

int main(int argc, char ** argv)
{
    // 初始化 ROS2 客户端
    rclcpp::init(argc,argv);
    // 创建对象指针并调用其功能
    auto client = std::make_shared<ImageClient>();
    bool bFlag = client->connect_server();
    if (!bFlag)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hik_camera_image_client"),"[Client] SN:%s Wait server time out!", client->m_strCamSerialNumber.c_str());
        return 0;
    }
    bool bExit = false;
    // 开始取流
    client->camera.start();
    while(!bExit)
    {
        // 获取图像指针信息
        vector<sensor_msgs::msg::Image::Ptr> stImagePtr = client->camera.getImage();
        // 获取图像类型信息
        uint32_t nImgType = client->camera.getMsgImgType();
        // 组合消息包
        int64_t nTimeStamp = client->camera.getTimeStamp();
        uint32_t nFrameNum = client->camera.getFrameNum();
        auto response = client->send_request(stImagePtr, nTimeStamp, nImgType, nFrameNum);
        if(nImgType)
        {
            RCLCPP_INFO(rclcpp::get_logger("hik_camera_image_client"), "[Client] SN:%s Send image[%d] msgs success", client->m_strCamSerialNumber.c_str(), nFrameNum);
        }
        // 处理响应
        if (rclcpp::spin_until_future_complete(client,response) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(client->get_logger(),"[Client] SN:%s Response is not success", client->m_strCamSerialNumber.c_str());
            bExit = true;
        }
    }
    // 关闭相机
    client->camera.stop();
    client->camera.close();
    // 释放资源
    rclcpp::shutdown();
    return 0;
}
