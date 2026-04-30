#include "../common/hik_camera.h"
#include "mv3d_rgbd_ros2_interface/srv/point_cloud.hpp"

using mv3d_rgbd_ros2_interface::srv::PointCloud;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

class PointCloudServer : public rclcpp::Node
{
public:
    PointCloudServer():Node("hik_camera_point_cloud_server")
    {
        RCLCPP_INFO(this->get_logger(), "-------------------- Server start --------------------");
        mkdir("src/mv3d_rgbd_ros2/point_cloud", S_IRWXU);
        strSavePath = "src/mv3d_rgbd_ros2/point_cloud";
        RCLCPP_INFO(this->get_logger(), "[Subscriber] save path: %s", strSavePath.c_str());
        server = this->create_service<PointCloud>("SavePointCloud", std::bind(&PointCloudServer::savePointCloud, this, _1, _2));
    }
private:
    rclcpp::Service<PointCloud>::SharedPtr server;
    string strSavePath;
    void savePointCloud(const PointCloud::Request::SharedPtr request, const PointCloud::Response::SharedPtr response)
    {
        // 获取帧号
        uint32_t nFrameNum = request->frame_num;
        // 解析图像类型
        uint32_t nMsgPointCloudType = request->pd_type;
        // 获取相机序列号
        string strSerialNum = request->cam_sn;
        pcl::PCDWriter pcdWriter;
        char chFileName[256] = "";
        if(nMsgPointCloudType == TYPE_POINT_CLOUD)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
            // 把msg消息转化为点云
            pcl::fromROSMsg(request->point_cloud, *pclPointCloud);
            sprintf(chFileName, "%s/[%d]_point_cloud.pcd", strSavePath.c_str(), nFrameNum);
            pcdWriter.write(chFileName, *pclPointCloud, false);
            RCLCPP_INFO(this->get_logger(), "[Server] SN:%s save [%d]_point_cloud.pcd success", strSerialNum.c_str(), nFrameNum);
        }
        else if(nMsgPointCloudType == TYPE_POINT_CLOUD_WITH_NORMALS)
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr pclPointCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
            // 把msg消息转化为法向量点云
            pcl::fromROSMsg(request->point_cloud, *pclPointCloudWithNormals);
            sprintf(chFileName, "%s/[%d]_point_cloud_with_normals.pcd", strSavePath.c_str(), nFrameNum);
            pcdWriter.write(chFileName, *pclPointCloudWithNormals, false);
            RCLCPP_INFO(this->get_logger(), "[Server] SN:%s save [%d]_point_cloud_with_normals.pcd success", strSerialNum.c_str(), nFrameNum);
        }
        else if(nMsgPointCloudType == TYPE_TEXTURED_POINT_CLOUD)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclTexturedPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            // 把msg消息转化为纹理点云
            pcl::fromROSMsg(request->point_cloud, *pclTexturedPointCloud);
            sprintf(chFileName, "%s/[%d]_textured_point_cloud.pcd", strSavePath.c_str(), nFrameNum);
            pcdWriter.write(chFileName, *pclTexturedPointCloud, false);
            RCLCPP_INFO(this->get_logger(), "[Server] SN:%s save [%d]_textured_point_cloud.pcd success", strSerialNum.c_str(), nFrameNum);
        }
        else if(nMsgPointCloudType == TYPE_TEXTURED_POINT_CLOUD_WITH_NORMALS)
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclTexturedPointCloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            // 把msg消息转化为带法向量的纹理点云
            pcl::fromROSMsg(request->point_cloud, *pclTexturedPointCloudWithNormals);
            sprintf(chFileName, "%s/[%d]_textured_point_cloud_with_normals.pcd", strSavePath.c_str(), nFrameNum);
            pcdWriter.write(chFileName, *pclTexturedPointCloudWithNormals, false);
            RCLCPP_INFO(this->get_logger(), "[Server] SN:%s save [%d]_textured_point_cloud_with_normals.pcd success", strSerialNum.c_str(), nFrameNum);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[Server] SN:%s point cloud type[%d] error...", nMsgPointCloudType);
        }
    }

};

int main(int argc, char *argv[])
{
    // 初始化ros节点
    rclcpp::init(argc, argv);
    auto server = std::make_shared<PointCloudServer>();
    // 调用spin函数，并传入节点对象指针
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
}
