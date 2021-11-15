#include "cam_radar_calibration/radar_camera_reprojection.h"

Reprojection::Reprojection()
    : it_(nh_)
{
    // nh_.getParam("distortion_model", distortion_model);
    nh_.getParam("height", height);
    nh_.getParam("width", width);
    nh_.getParam("K", K);
    nh_.getParam("D", D);


    // Load in camera_info to cv::Mat
    cameramat = cv::Mat::zeros(3, 3, CV_64F);
    distcoeff = cv::Mat::eye(1, 4, CV_64F);
    cameramat.at<double>(0, 0) = K[0];
    cameramat.at<double>(0, 2) = K[2];
    cameramat.at<double>(1, 1) = K[4];
    cameramat.at<double>(1, 2) = K[5];
    cameramat.at<double>(2, 2) = 1;

    distcoeff.at<double>(0) = D[0];
    distcoeff.at<double>(1) = D[1];
    distcoeff.at<double>(2) = D[2];
    distcoeff.at<double>(3) = D[3];

    img_sub_.subscribe(nh_, "camera_1/image_raw", 1);
    pcl_sub_.subscribe(nh_, "oculli_radar_pcl/rcs", 1);
    extrinsic_calib_param_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>
                                ("extrinsic_calib_param", 1, &Reprojection::paramsCallback, this);

    sync_.reset(new Sync(MySyncPolicy(10), img_sub_, pcl_sub_));
    sync_->registerCallback(boost::bind(&Reprojection::syncCallback, this, _1, _2));

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    cam_radar_img_pub_ = it_.advertise("radar_camera_img", 1);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl", 1);

    std::cout << "init complete" << std::endl;  
}

void 
Reprojection::syncCallback(const ImgConstPtr &img_msg, const PclConstPtr &pcl_msg) 
{
    std::cout << "image timestamp: " << img_msg->header.stamp << std::endl;
    std::cout << "pcl timestamp  : " << pcl_msg->header.stamp << std::endl;

    auto cloud_msg = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto appended_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    pcl::fromROSMsg(*pcl_msg, *cloud_msg);

    for (auto& point : *cloud_msg) {
        for (double i = 0.0; i < 1.1; i += 0.1) { // -0.85 < 0.3
            pcl::PointXYZI pt;
            pt.x = point.x;
            pt.y = point.y;
            pt.z = i;
            pt.intensity = point.intensity;
            appended_cloud->points.push_back(pt);
        }
    }

    geometry_msgs::TransformStamped transformStamped;
    Eigen::Affine3f tr_out;
    float theta = M_PI;
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    try {
        transformStamped = tf_buffer_.lookupTransform("camera_1_link", "oculli_link", 
                                                        ros::Time::now(), ros::Duration(1.0));
        const auto &t = transformStamped.transform.translation;
        const auto &q = transformStamped.transform.rotation;
        tr_out = Eigen::Translation3f(t.x, t.y, t.z) * Eigen::Quaternionf(q.w, q.x, q.y, q.z);

        
        transform_2.translation() << t.x, t.y, t.z;
        transform_2.rotate(Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
        std::cout << "here" << std::endl;
    }
    catch (tf2::TransformException &ex) {
        printf("TF Query Failed %s -> %s\n", "camera_1_link", "oculli_link");
    }

    //pcl::transformPointCloud(*appended_cloud, *appended_cloud, tr_out);
    pcl::transformPointCloud(*appended_cloud, *appended_cloud, transform_2);

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;

    std::cout << "cloud size: " << cloud_msg->points.size() << std::endl;

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(appended_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (2.0, 30.0);
    pass.filter (*appended_cloud);

    pass.setInputCloud(appended_cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-3.0, 3.0);
    pass.filter (*appended_cloud);

    //std::cout << "filtered cloud size: " << cloud_filtered->points.size() << std::endl;

    if (appended_cloud->points.size()) {
        // for (auto& point : *appended_cloud) {
        //     for (double i = 0.0; i < 1.1; i += 0.1) { // -0.85 < 0.3
        //         pcl::PointXYZI pt;
        //         pt.x = point.x;
        //         pt.y = point.y;
        //         pt.z = i;
        //         pt.intensity = point.intensity;
        //         appended_cloud->points.push_back(pt);
        //     }
        // }

        std::cout << "new cloud size: " << appended_cloud->points.size() << std::endl;

        for (pcl::PointCloud<pcl::PointXYZI>::const_iterator it = appended_cloud->begin(); it != appended_cloud->end(); it++) {
            double tmpxC = (it->y * -1) / it->x;
            double tmpyC = (it->z * -1) / it->x;
            double tmpzC = it->x;

            // double tmpxC = it->x / it->z;
            // double tmpyC = it->y / it->z;
            // double tmpzC = it->z;
            double dis = pow(it->x * it->x + it->y * it->y + it->z * it->z, 0.5);
            cv::Point2d planepointsC;
            int range = std::min(round((dis / 30.0) * 49), 49.0);

            // Applying the distortion
            double r2 = tmpxC * tmpxC + tmpyC * tmpyC;
            double r1 = pow(r2, 0.5);
            double a0 = std::atan(r1);
            double a1;
            a1 = a0 * (1 + distcoeff.at<double>(0) * pow(a0, 2) + distcoeff.at<double>(1) * pow(a0, 4) +
                    distcoeff.at<double>(2) * pow(a0, 6) + distcoeff.at<double>(3) * pow(a0, 8));
            // planepointsC.x = (a1 / r1) * tmpxC;
            // planepointsC.y = (a1 / r1) * tmpyC;

            planepointsC.x = tmpxC;
            planepointsC.y = tmpyC;

            planepointsC.x = cameramat.at<double>(0, 0) * planepointsC.x + cameramat.at<double>(0, 2);
            planepointsC.y = cameramat.at<double>(1, 1) * planepointsC.y + cameramat.at<double>(1, 2);

            // std::cout << "radar pt [x,y,z]: [" << it->x << "," << it->y << "," << it->z << "] ---> ";
            // std::cout << "point position [x,y]: [" << planepointsC.x << "," << planepointsC.y << "]"<< std::endl;  
            // std::cout << "tmpxC: " << tmpxC << std::endl;
            std::cout << "point rcs: " << it->intensity << std::endl;
            

            if (planepointsC.y >= 0 and planepointsC.y < height and planepointsC.x >= 0 and planepointsC.x < width and
                tmpzC >= 0 and std::abs(tmpxC) <= 1.35) {

                int point_size = 8;
                cv::circle(cv_ptr->image,
                    cv::Point(planepointsC.x, planepointsC.y), point_size,
                    CV_RGB(255 * colmap[50-range][0], 255 * colmap[50-range][1], 255 * colmap[50-range][2]), -1);
            }
            // cv::circle(cv_ptr->image,
            //         cv::Point(350, 50), 10,
            //         CV_RGB(230, 16, 194), -1);
        }
    }
    // ROS_INFO_STREAM("Projecting points onto image for pose #" << (visualise_pose_num));
    // compute_reprojection(sample_list[visualise_pose_num-1], cam_project, lidar_project);
    // for (auto& point : cam_project)
    // {
    //     cv::circle(image, point, 15, CV_RGB(0, 255, 0), 2);
    //     cv::drawMarker(image, point, CV_RGB(0,255,0), cv::MARKER_CROSS, 25, 2, cv::LINE_8);
    // }
    // for (auto& point : lidar_project)
    // {
    //     cv::circle(image, point, 15, CV_RGB(255, 255, 0), 2);
    //     cv::drawMarker(image, point, CV_RGB(255,255,0), cv::MARKER_TILTED_CROSS, 20, 2, cv::LINE_8);
    // }

    cam_radar_img_pub_.publish(cv_ptr->toImageMsg());

    pcl_pub_.publish(pcl_msg);
    std::cout << std::endl;

}

void
Reprojection::paramsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) 
{
    // Need to inverse the transforms
    // In calibration, we figured out the transform to make camera into lidar frame (here we do opposite)
    // Here we apply transform to lidar i.e. tf(lidar) (parent) -----> camera (child)
    tf2::Transform transform;
    tf2::Quaternion quat;
    tf2::Vector3 trans;
    quat.setRPY(msg->data[0],msg->data[1],msg->data[2]);
    trans.setX(msg->data[3]);
    trans.setY(msg->data[4]);
    trans.setZ(msg->data[5]);
    transform.setRotation(quat);
    transform.setOrigin(trans);
    // ROS_INFO("Inverting rotation and translation for projecting LiDAR points into camera image");
    tf_msg.transform.rotation.w = transform.inverse().getRotation().w();
    tf_msg.transform.rotation.x = transform.inverse().getRotation().x();
    tf_msg.transform.rotation.y = transform.inverse().getRotation().y();
    tf_msg.transform.rotation.z = transform.inverse().getRotation().z();
    tf_msg.transform.translation.x = transform.inverse().getOrigin().x();
    tf_msg.transform.translation.y = transform.inverse().getOrigin().y();
    tf_msg.transform.translation.z = transform.inverse().getOrigin().z();

    double r_val,y_val,p_val;
    double d1,d2,d3;
    geometry_msgs::Quaternion q = tf_msg.transform.rotation;
    tf::Quaternion tfq;
    tf::quaternionMsgToTF(q, tfq);
    tf::Matrix3x3(tfq).getEulerYPR(y_val,p_val,r_val);
    rot_trans.x = tf_msg.transform.translation.x * 1000;
    rot_trans.y = tf_msg.transform.translation.y * 1000;
    rot_trans.z = tf_msg.transform.translation.z * 1000;
    rot_trans.rot.roll = r_val;
    rot_trans.rot.pitch = p_val;
    rot_trans.rot.yaw = y_val;
}

int 
main(int argc, char **argv)
{
    ros::init(argc, argv, "reprojection");

    Reprojection reprojection;

    ros::spin();
    return 0;
}
