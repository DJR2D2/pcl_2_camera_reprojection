#ifndef REPROJECTION_NODE_H_
#define REPROJECTION_NODE_H_

// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64MultiArray.h>
#include <image_transport/image_transport.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>

#include <iostream>
#include <string>
#include <numeric>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/transforms.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"

double colmap[50][3]={{0,0,0.5385},{0,0,0.6154},{0,0,0.6923},{0,0,0.7692},{0,0,0.8462},{0,0,0.9231},{0,0,1.0000},{0,0.0769,1.0000},{0,0.1538,1.0000},{0,0.2308,1.0000},{0,0.3846,1.0000},
                      {0,0.4615,1.0000},{0,0.5385,1.0000},{0,0.6154,1.0000},{0,0.6923,1.0000},{0,0.7692,1.0000},{0,0.8462,1.0000},{0,0.9231,1.0000},{0,1.0000,1.0000},{0.0769,1.0000,0.9231},
                      {0.1538,1.0000,0.8462},{0.2308,1.0000,0.7692},{0.3077,1.0000,0.6923},{0.3846,1.0000,0.6154},{0.4615,1.0000,0.5385},{0.5385,1.0000,0.4615},{0.6154,1.0000,0.3846},
                      {0.6923,1.0000,0.3077},{0.7692,1.0000,0.2308},{0.8462,1.0000,0.1538},{0.9231,1.0000,0.0769},{1.0000,1.0000,0},{1.0000,0.9231,0},{1.0000,0.8462,0},{1.0000,0.7692,0},
                      {1.0000,0.6923,0},{1.0000,0.6154,0},{1.0000,0.5385,0},{1.0000,0.4615,0},{1.0000,0.3846,0},{1.0000,0.3077,0},{1.0000,0.2308,0},{1.0000,0.1538,0},{1.0000,0.0769,0},
                      {1.0000,0,0},{0.9231,0,0},{0.8462,0,0},{0.7692,0,0},{0.6923,0,0},{0.6154,0,0}};

struct OptimisationSample
{
    cv::Point3d camera_centre{ 0, 0, 0 };
    cv::Point3d camera_normal{ 0, 0, 0 };
    std::vector<cv::Point3d> camera_corners;
    cv::Point3d lidar_centre{ 0, 0, 0 };
    cv::Point3d lidar_normal{ 0, 0, 0 };
    std::vector<cv::Point3d> lidar_corners;
    std::vector<double> angles_0;
    std::vector<double> angles_1;
    std::vector<double> widths;
    std::vector<double> heights;
    float distance_from_origin;
    double pixeltometre;
    int sample_num;
};

struct Rotation
{
    double roll;  // Rotation optimization variables
    double pitch;
    double yaw;
    operator const std::string() const
    {
        return std::string("{") + "roll:" + std::to_string(roll) + ", pitch:" + std::to_string(pitch) +
                ", yaw:" + std::to_string(yaw) + "}";
    }
    cv::Mat toMat() const
    {
        using cv::Mat_;
        using std::cos;
        using std::sin;

        // Calculate rotation about x axis
        cv::Mat R_x = (Mat_<double>(3, 3) << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll));
        // Calculate rotation about y axis
        cv::Mat R_y = (Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch));
        // Calculate rotation about z axis
        cv::Mat R_z = (Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1);

        return R_z * R_y * R_x;
    }
};

struct RotationTranslation
{
    Rotation rot;
    double x;
    double y;
    double z;
    operator const std::string() const
    {
        return std::string("{") + "roll:" + std::to_string(rot.roll) + ", pitch:" + std::to_string(rot.pitch) +
                ", yaw:" + std::to_string(rot.yaw) + ", x:" + std::to_string(x) + ", y:" + std::to_string(y) +
                ", z:" + std::to_string(z) + "}";
    }
};

cv::Mat operator*(const Rotation& lhs, const cv::Point3d& rhs)
{
    cv::Mat mat = cv::Mat(rhs).reshape(1);
    // Combined rotation matrix
    return lhs.toMat() * mat;
}
cv::Mat operator*(const RotationTranslation& lhs, const cv::Point3d& rhs)
{
    auto rotated = cv::Point3d(lhs.rot * rhs);
    rotated.x += lhs.x;
    rotated.y += lhs.y;
    rotated.z += lhs.z;
    return cv::Mat(rotated).reshape(1);
}


class Reprojection
{
public:
    Reprojection();

    typedef boost::shared_ptr< ::sensor_msgs::Image const> ImgConstPtr;
    typedef boost::shared_ptr< ::sensor_msgs::PointCloud2 const> PclConstPtr;

    void syncCallback(const ImgConstPtr &img_msg, const PclConstPtr &pcl_msg);
    void paramsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

    std::vector<double> params_; 

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;

    typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;

    ImageSubscriber img_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub_;
    boost::shared_ptr<Sync> sync_;

    image_transport::Publisher cam_radar_img_pub_;
    ros::Publisher pcl_pub_;

    ros::Subscriber extrinsic_calib_param_sub_;   
    std_msgs::Float64MultiArray::ConstPtr param_msg_;
    std::vector<OptimisationSample> sample_list_;
    geometry_msgs::TransformStamped tf_msg;
    RotationTranslation rot_trans;

    std::string distortion_model;
    std::vector<double> K, D;
    cv::Mat cameramat, distcoeff;
    int height, width;
    cv::Size board_dimensions;

};

#endif 