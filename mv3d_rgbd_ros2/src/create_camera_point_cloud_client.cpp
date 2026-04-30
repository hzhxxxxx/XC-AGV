#include "../common/hik_camera.h"
#include "mv3d_rgbd_ros2_interface/srv/point_cloud.hpp"

using mv3d_rgbd_ros2_interface::srv::PointCloud;
using namespace std;
using namespace std::chrono_literals;

// 定义节点类
class PointCloudClient: public rclcpp::Node{
public:
    PointCloudClient():Node("hik_camera_point_cloud_client")
    {
        this->declare_parameter<std::string>("cam_sn", "None");
        this->declare_parameter<int>("pd_type", 1);
        RCLCPP_INFO(this->get_logger(),"-------------------- Client start --------------------");
        // 初始化相机并连接
        camera.initialize();
        // 获取点云类型
        int nPointCloudType = 1;
        this->get_parameter("pd_type", nPointCloudType);
        if(nPointCloudType < 1 || nPointCloudType > 4)
        {
            nPointCloudType = 1;
            RCLCPP_INFO(this->get_logger(), "point cloud type[%d] is not valid, used default type[0]", nPointCloudType);
        }
 
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
            RCLCPP_INFO(rclcpp::get_logger("hik_camera_point_cloud_client"), "---------------- Connect by serial number: %s ----------------", strSerialNumber.c_str());
            // 根据序列号进行连接
            camera.connect(strSerialNumber.c_str());
        }
        // 获取相机序列号
        m_strCamSerialNumber = camera.getSerialNumber();
        // 设置点云输出
        MV3D_RGBD_PARAM stParam;
        stParam.enParamType = ParamType_Enum;
        stParam.ParamInfo.stEnumParam.nCurValue = nPointCloudType;
        int nRet = camera.setParam(MV3D_RGBD_ENUM_POINT_CLOUD_OUTPUT, &stParam);
        if(MV3D_RGBD_OK != nRet)
        {
            RCLCPP_ERROR(this->get_logger(), "[Client] SN:%s Set point cloud type[%d] failed...", m_strCamSerialNumber.c_str(), nPointCloudType);
            camera.close();
        }
        else
        {
            // 创建客户端
            client = this->create_client<PointCloud>("SavePointCloud");
        }
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
    rclcpp::Client<PointCloud>::FutureAndRequestId send_request(sensor_msgs::msg::PointCloud2& stPointCloudMsg,
    int64_t nTimeStamp, uint32_t nPdType, uint32_t nFrameNum)
    {
        auto request = std::make_shared<PointCloud::Request>();
        request->cam_sn = m_strCamSerialNumber;
        request->time_stamp = nTimeStamp;
        request->pd_type = nPdType;
        request->frame_num = nFrameNum;
        request->point_cloud = stPointCloudMsg;
        return client->async_send_request(request);
    }

    HikCamera camera;
    string m_strCamSerialNumber;
private:
    rclcpp::Client<PointCloud>::SharedPtr client;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char ** argv)
{
    // 初始化 ROS2 客户端
    rclcpp::init(argc,argv);
    // 创建对象指针并调用其功能
    auto client = std::make_shared<PointCloudClient>();
    bool bFlag = client->connect_server();
    if (!bFlag)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hik_camera_point_cloud_client"),"[Client] SN:%s Wait server time out!", client->m_strCamSerialNumber.c_str());
        return 0;
    }
    bool bExit = false;
    // 开始取流
    client->camera.start();
    while(!bExit && rclcpp::ok())
    {
        // 获取点云信息
        sensor_msgs::msg::PointCloud2 stPointCloudMsg = client->camera.getPointCloud();
        // 获取图像类型信息
        uint32_t nPdType = client->camera.getMsgPointCloudType();
        // 组合消息包
        int64_t nTimeStamp = client->camera.getTimeStamp();
        uint32_t nFrameNum = client->camera.getFrameNum();
        if(nPdType)
        {
            auto response = client->send_request(stPointCloudMsg, nTimeStamp, nPdType, nFrameNum);
            RCLCPP_INFO(rclcpp::get_logger("hik_camera_point_cloud_client"), "[Client] SN:%s Send point cloud[%d] msgs success", client->m_strCamSerialNumber.c_str(), nFrameNum);
            // 处理响应
            if (rclcpp::spin_until_future_complete(client,response) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(client->get_logger(),"[Client] SN:%s Response is not success", client->m_strCamSerialNumber.c_str());
                bExit = true;
            }
        }
    }
    // 关闭相机
    client->camera.stop();
    client->camera.close();
    // 释放资源
    rclcpp::shutdown();
    return 0;
}
