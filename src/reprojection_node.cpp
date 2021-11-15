#include "cam_radar_calibration/radar_camera_reprojection.h"

using radar_cam_reprojection::Reprojection;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_radar_reprojection");
    Reprojection reprojection;
    ros::spin();
    return 0;
}

