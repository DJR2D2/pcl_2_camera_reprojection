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

    global_pcl_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    std::cout << "init complete" << std::endl;  
}

void 
Reprojection::syncCallback(const ImgConstPtr &img_msg, const PclConstPtr &pcl_msg) 
{
    std::cout << "image timestamp: " << img_msg->header.stamp << std::endl;
    std::cout << "pcl timestamp  : " << pcl_msg->header.stamp << std::endl;

    auto cloud_msg = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto appended_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // ######### Static Transforms ###########
    geometry_msgs::TransformStamped tf;
    geometry_msgs::TransformStamped tf_radar;
    tf = tf_buffer_.lookupTransform("camera_1_link", "oculli_link",
                                    ros::Time::now(),
                                    ros::Duration(1.0));

    tf_radar = tf_buffer_.lookupTransform("base_link", "oculli_link", // this is only needed for viewing the radar pcl in reference to odom.
                                    ros::Time::now(),
                                    ros::Duration(1.0));

    sensor_msgs::PointCloud2 cloud_tf;
    tf2::doTransform(*pcl_msg, cloud_tf, tf_radar);
    // pcl::fromROSMsg(cloud_tf, *cloud_msg);
    pcl::fromROSMsg(*pcl_msg, *cloud_msg);

    // ######### Get Perspective Transform #############
    cv_bridge::CvImagePtr cv_ptr;//, cv_img_pub;
    cv_bridge::CvImage cv_img_pub;
    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;
    std::cout << "address of image: " << &image << std::endl;
    std::cout << "address of cv_ptr->image: " << &cv_ptr->image << std::endl;
    // cv::Mat pt_image(image.rows, image.cols, CV_8UC3);
    cv::Mat pt_image = cv::Mat::zeros(cv::Size(image.cols, image.rows), CV_8UC3);

    // image_buf_ = cv_ptr->image;
    // std::cout << "image type: " << image.type() << std::endl;
    cv::Mat temp_img;

    // ######### Incoming PCL filter ################
    std::cout << "cloud size: " << cloud_msg->points.size() << std::endl;

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud_msg);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.5, 30.0);
    pass.filter (*cloud_filtered);

    std::cout << "filtered x cloud size: " << cloud_filtered->points.size() << std::endl;

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-2.0, 2.0);
    pass.filter (*cloud_msg);

    std::cout << "filtered y cloud size: " << cloud_filtered->points.size() << std::endl;

    pass.setInputCloud(cloud_msg);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-3.0, 0.0);
    pass.filter (*cloud_filtered);

    std::cout << "filtered z cloud size: " << cloud_filtered->points.size() << std::endl;

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setLeafSize(1.0f, 1.0f, 1.0f);
    sor.filter(*cloud_filtered);

    std::cout << "filtered vg cloud size: " << cloud_filtered->points.size() << std::endl;

    // auto left_hist = make_histogram(axis::regular<>(10, 0.0, 2.0));
    // auto right_hist = make_histogram(axis::regular<>(10, -2.0, 0.0));

    if (cloud_filtered->points.size()) {
        // for (auto& point : *cloud_filtered) {
        //     left_hist(point.y);
        //     right_hist(point.y);
        // }

        double rcs_min {100.0};
        double rcs_max {-100.0};
        double range_max {-100.0};
        double helper_var {0.0};
        double rcs_mean {0.0};
        double rcs_sd {0.0};
        int rcs_count {0};

        // size_t left_lar_bin {0};
        // size_t right_lar_bin {0};
        // auto left_bin_max {0.};
        // auto left_bin_min {0.};
        // auto right_bin_max {0.};
        // auto right_bin_min {0.};
        // for (auto&& x : indexed(left_hist)) {
        //     if (*x > left_lar_bin) {
        //         left_lar_bin = *x;
        //         left_bin_max = x.bin().upper();
        //         left_bin_min = x.bin().lower();             
        //     }
        //     std::cout << boost::format("bin %i [ %.1f, %.1f ): %i\n")
        //         % x.index() % x.bin().lower() % x.bin().upper() % *x;
        // }
        // std::cout << std::flush;

        // for (auto&& x : indexed(right_hist)) {
        //     if (*x > right_lar_bin) {
        //         right_lar_bin = *x;
        //         right_bin_max = x.bin().upper();
        //         right_bin_min = x.bin().lower();             
        //     }
        //     std::cout << boost::format("bin %i [ %.1f, %.1f ): %i\n")
        //         % x.index() % x.bin().lower() % x.bin().upper() % *x;
        // }
        // std::cout << std::flush;

        // std::cout << "points in left largest bin : " << left_lar_bin << std::endl;
        // std::cout << "points in right largest bin: " << right_lar_bin << std::endl;

        for (auto& point : *cloud_filtered) {
            // if ((point.y >= left_bin_min && point.y <= left_bin_max) || (point.y >= right_bin_min && point.y <= right_bin_max)) {
            //     for (double i = -0.856; i < 0.65; i += 0.1) { // -0.856 < 0.29
            //         pcl::PointXYZI pt;
            //         pt.x = point.x;
            //         pt.y = point.y;
            //         pt.z = i;
            //         pt.intensity = point.intensity;
            //         appended_cloud->points.push_back(pt);           
            //     }

            //     rcs_min = (point.intensity < rcs_min) ? point.intensity : rcs_min;
            //     rcs_max = (point.intensity > rcs_max) ? point.intensity : rcs_max;
            //     helper_var = sqrt(pow(point.x, 2) + pow(point.y, 2));
            //     range_max = (helper_var > range_max) ? helper_var : range_max;
            //     rcs_mean += point.intensity;
            //     rcs_count++;  
            // }

            rcs_min = (point.intensity < rcs_min) ? point.intensity : rcs_min;
            rcs_max = (point.intensity > rcs_max) ? point.intensity : rcs_max;
            helper_var = sqrt(pow(point.x, 2) + pow(point.y, 2));
            range_max = (helper_var > range_max) ? helper_var : range_max;
            rcs_mean += point.intensity;
            rcs_count++;
        }

        std::vector<cv::Point2f> local_pt_buf;
        
        double rcs_range = rcs_max - rcs_min;
        rcs_mean /= rcs_count;

        std::cout << "rcs range: " << rcs_range << std::endl;
        std::cout << "distance range: " << range_max << std::endl;
        std::cout << "rcs mean: " << rcs_mean << std::endl;

        // ############### TRIAL FILTERING #####################

        for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud_filtered->begin(); it != cloud_filtered->end(); it++) {
            std::cout << "original: " << it->intensity << std::endl;
            std::cout << "distance from radar: " << sqrt(pow(it->x, 2) + (pow(it->y, 2))) << std::endl;
            std::cout << "X: " << it->x << std::endl;
            std::cout << "Y: " << it->y << std::endl;
            std::cout << "Z: " << it->z << std::endl;
            // it->intensity = it->intensity * (((sqrt(pow(it->x, 2) + (pow(it->y, 2))))) / range_max);
            it->intensity = (it->intensity - rcs_mean) / rcs_range;
            it->intensity = 1 / (1 + exp(-20 * it->intensity + 0));
            std::cout << "modified: " << it->intensity << std::endl;
            std::cout << std::endl;
        }

        // pass.setInputCloud(cloud_filtered);
        // pass.setFilterFieldName("intensity");
        // pass.setFilterLimits(0.5, 1.0);
        // pass.filter(*cloud_filtered);
        

        // ########### Convert PCL to image coordinates ##########################
        // for (pcl::PointCloud<pcl::PointXYZI>::const_iterator it = cloud_filtered->begin(); it != cloud_filtered->end(); it++) {
        for (pcl::PointCloud<pcl::PointXYZI>::const_iterator it = appended_cloud->begin(); it != appended_cloud->end(); it++) {
            double tmpxC = (it->y * -1) / it->x;
            double tmpyC = (it->z * -1) / it->x;
            double tmpzC = it->x;

            // double tmpxC = it->x / it->z;
            // double tmpyC = it->y / it->z;
            // double tmpzC = it->z;
            double dis = pow(it->x * it->x + it->y * it->y + it->z * it->z, 0.5);
            cv::Point2d planepointsC;
            // int range = std::min(round((dis / 30.0) * 49), 49.0);


            int range = std::min(round((abs(it->intensity) / abs(rcs_max - rcs_min)) * 49), 49.0);
            // std::cout << "range = " << range << std::endl;

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
            // std::cout << "point rcs: " << it->intensity << std::endl;
            

            if (planepointsC.y >= 0 and planepointsC.y < height and planepointsC.x >= 0 and planepointsC.x < width and
                tmpzC >= 0 and std::abs(tmpxC) <= 1.35) {

                int point_size = 4;

                local_pt_buf.push_back(Point2f(planepointsC.x, planepointsC.y));

                cv::circle(cv_ptr->image,
                    cv::Point(planepointsC.x, planepointsC.y), point_size,
                    CV_RGB(245, 218, 66), -1);

                // cv::circle(pt_image,
                //     cv::Point(planepointsC.x, planepointsC.y), point_size,
                //     CV_RGB(245, 23, 23), -1);
            }

        }
        

        // if (pt_buf_.size() != 0) {
        //     Mat lambda(2, 4, CV_32FC1);
        //     Mat tmp_img;
        //     lambda = featureDetector(image_buf_, cv_ptr->image);
        //     std::cout << "lambda: " << lambda << std::endl;
        //     for (size_t i = 0; i < pt_buf_.size(); i++) {
        //         for (size_t j = 0; j < pt_buf_[i].size(); j++) {
        //             cv::Mat_<double> src(3/*rows*/,1 /* cols */); 

        //             src(0,0)=pt_buf_[i][j].x; 
        //             src(1,0)=pt_buf_[i][j].y; 
        //             src(2,0)=1.0; 

        //             cv::Mat_<double> dst = lambda*src; //USE MATRIX ALGEBRA
        //             pt_buf_[i][j] = cv::Point2f(dst(0,0), dst(1,0));

        //             cv::circle(cv_ptr->image,
        //                 cv::Point(pt_buf_[i][j].x, pt_buf_[i][j].y), 4.0,
        //                 CV_RGB(245, 218, 66), -1);

        //             // std::cout << "input" << i << " : " << pt_buf_[i][i] << std::endl;
        //             // std::cout << "output" << i << ": " << cv::Point2f(dst(0,0), dst(1,0)) << std::endl;

        //     }
        //         // cv::warpPerspective(img_buf_[i], img_buf_[i], lambda, img_buf_[i].size());
        //         // addWeighted(image, 1.0, img_buf_[i], 1.0, 0.0, image);
        //     }
        // }


        // ################ Radar to Image to Reprojection ##################
        

        // ########### Transform PCL buffer ################################



        // ########### add pcl_mat to que ##########################
        image_buf_ = image;
        pt_buf_.push_back(local_pt_buf);
        if (pt_buf_.size() > 10)
            pt_buf_.pop_front();
    }

    // ################ Publishers ##########################
    // cam_radar_img_pub_.publish(cv_img_pub.toImageMsg());
    cam_radar_img_pub_.publish(cv_ptr->toImageMsg());

    sensor_msgs::PointCloud2 pcl_tf;
    pcl::toROSMsg(*cloud_filtered, pcl_tf);
    // tf2::doTransform(*pcl_msg, pcl_tf, tf_radar);
    pcl_tf.header = pcl_msg->header;
    pcl_pub_.publish(pcl_tf);
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

cv::Mat
Reprojection::featureDetector(cv::Mat img_1, cv::Mat img_2)
{
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");


    detector->detect(img_1,keypoints_1);
    detector->detect(img_2,keypoints_2);


    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    Mat outimg1;
    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // imshow("ORB特征点",outimg1);


    vector<DMatch> matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match (descriptors_1, descriptors_2, matches);

    double min_dist=10000, max_dist=0;

    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    std::vector< DMatch > good_matches;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if ( matches[i].distance <= max(2*min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    // Point2f img1_pts[good_matches.size()];
    // Point2f img2_pts[good_matches.size()];

    // Mat lambda(2, 4, CV_32FC1);
    // lambda = Mat::zeros(img_1.rows, img_1.cols, img_1.type());

    // for(size_t i = 0; i < good_matches.size(); i++) {
    //     img1_pts[i] = Point2f(keypoints_1.at(good_matches[i].queryIdx).pt.x, keypoints_1.at(good_matches[i].queryIdx).pt.y);
    //     img2_pts[i] = Point2f(keypoints_2.at(good_matches[i].trainIdx).pt.x, keypoints_2.at(good_matches[i].trainIdx).pt.y);
    // }

    // lambda = getPerspectiveTransform(img1_pts, img2_pts);

    Point2f img1_pts[good_matches.size()];
    Point2f img2_pts[good_matches.size()];

    Mat lambda(2, 4, CV_32FC1);
    lambda = Mat::zeros(img_1.rows, img_1.cols, img_1.type());

    std::vector<cv::Point2f> yourPoints1;
    std::vector<cv::Point2f> yourPoints2;
    cv::Mat pt_img1 = cv::Mat::zeros(cv::Size(img_1.cols, img_1.rows), CV_8UC3);
    cv::Mat pt_img2 = cv::Mat::zeros(cv::Size(img_1.cols, img_1.rows), CV_8UC3);


    for(size_t i = 0; i < good_matches.size(); i++) {
        yourPoints1.push_back(Point2f(keypoints_1.at(good_matches[i].queryIdx).pt.x, keypoints_1.at(good_matches[i].queryIdx).pt.y));
        yourPoints2.push_back(Point2f(keypoints_2.at(good_matches[i].trainIdx).pt.x, keypoints_2.at(good_matches[i].trainIdx).pt.y));
    }

    lambda = findHomography(yourPoints1, yourPoints2, RANSAC, 9.0);
    // Mat dst_mat = Mat::zeros(img_1.rows, img_1.cols, img_1.type());
    // cout << "lambda = " << endl << " " << lambda << endl << endl;

    // for (size_t i = 0; i < yourPoints1.size(); i++) {
    //     cv::Mat_<double> src(3/*rows*/,1 /* cols */); 

    //     src(0,0)=yourPoints1[i].x; 
    //     src(1,0)=yourPoints1[i].y; 
    //     src(2,0)=1.0; 

    //     cv::Mat_<double> dst = lambda*src; //USE MATRIX ALGEBRA
    //     // yourPoints1[i] = cv::Point2f(dst(0,0), dst(1,0));

    //     std::cout << "input" << i << " : " << yourPoints1[i] << std::endl;
    //     std::cout << "output" << i << ": " << cv::Point2f(dst(0,0), dst(1,0)) << std::endl;

    //     cv::circle(img_2,
    //                 cv::Point(cv::Point2f(dst(0,0), dst(1,0))), 3,
    //                 CV_RGB(245, 23, 23), -1);
    // }

    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    // imshow ( "img_match", img_match );
    // imshow ( "img_goodmatch", img_goodmatch );
    // waitKey(0);
    return lambda;
}



int 
main(int argc, char **argv)
{
    ros::init(argc, argv, "reprojection");

    Reprojection reprojection;

    ros::spin();
    return 0;
}
