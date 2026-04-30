#include "../common/hik_camera.h"

using namespace std;

class ImagePublish : public rclcpp::Node
{
public:
    ImagePublish() : Node("hik_camera_image_publisher"),strSerialNumber("None"),nSeq(0)
    {
        this->declare_parameter<std::string>("cam_sn", "None");
        depthPub = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth", rclcpp::QoS(10).reliable());
        rgbPub = this->create_publisher<sensor_msgs::msg::Image>("/camera/rgb", rclcpp::QoS(10).reliable());
        leftIrPub = this->create_publisher<sensor_msgs::msg::Image>("/camera/leftIr", rclcpp::QoS(10).reliable());
        rightIrPub = this->create_publisher<sensor_msgs::msg::Image>("/camera/rightIr", rclcpp::QoS(10).reliable());
        depthCamInfoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", rclcpp::QoS(10).reliable());
        rgbCamInfoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/rgb/camera_info", rclcpp::QoS(10).reliable());   
        // 获取序列号
        this->get_parameter("cam_sn", strSerialNumber);
        camera.initialize();
        if(strSerialNumber == "None")
        {
            // 默认连接第一个相机
            camera.connect();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "---------------- Connect by serial number: %s ----------------", strSerialNumber.c_str());
            // 根据序列号进行连接
            camera.connect(strSerialNumber.c_str());
        }
        // 获取标定信息
        stCamInfo = camera.getCamInfo();
        // 开始取流
        camera.start();
        timer = this->create_wall_timer(30ms, std::bind(&ImagePublish::publishImage, this));
    }
    
    HikCamera camera;
private:
    void publishImage()
    {
        // 获取图像指针信息
        vector<sensor_msgs::msg::Image::Ptr> stImagePtr = camera.getImage();
        // 获取图像类型信息
        uint32_t nImgType = camera.getMsgImgType();
        // 发布节点
        if(nImgType & TYPE_DEPTH)
        {
            depthPub->publish(*stImagePtr[0]);
            RCLCPP_INFO(this->get_logger(), "[Publisher] Send depth image [%d] success", nSeq);
        }
        if(nImgType & TYPE_RGB)
        {
            rgbPub->publish(*stImagePtr[1]);
            RCLCPP_INFO(this->get_logger(), "[Publisher] Send rgb image [%d] success", nSeq);
        }
        if(nImgType & TYPE_LEFT_IR)
        {
            leftIrPub->publish(*stImagePtr[2]);
            RCLCPP_INFO(this->get_logger(), "[Publisher] Send left ir image [%d] success", nSeq);
        }
        if(nImgType & TYPE_RIGHT_IR)
        {
            rightIrPub->publish(*stImagePtr[3]);
            RCLCPP_INFO(this->get_logger(), "[Publisher] Send right ir image [%d] success", nSeq);
        }
        stCamInfo[0].header.stamp = this->now(); depthCamInfoPub->publish(stCamInfo[0]);
        stCamInfo[1].header.stamp = this->now(); rgbCamInfoPub->publish(stCamInfo[1]);
        nSeq++;
    }
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftIrPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightIrPub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depthCamInfoPub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgbCamInfoPub;
    vector<sensor_msgs::msg::CameraInfo> stCamInfo; 
    rclcpp::TimerBase::SharedPtr timer;
    string strSerialNumber;
    uint32_t nSeq;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublish>();
    rclcpp::spin(node);
    node->camera.stop();
    node->camera.close();
    rclcpp::shutdown();
    return 0;
}