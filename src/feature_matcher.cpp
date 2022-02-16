#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// we're NOT "using namespace std;" here, to avoid collisions between the beta variable and std::beta in c++17
// using std::cin;
// using std::cout;
// using std::endl;

int main (int argc, char** argv)
{
    if (argc != 3)
    {
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }

    Mat img_1 = imread(argv[1], IMREAD_COLOR);
    Mat img_2 = imread(argv[2], IMREAD_COLOR);


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

    Point2f img1_pts[good_matches.size()];
    Point2f img2_pts[good_matches.size()];

    Mat lambda(2, 4, CV_32FC1);
    lambda = Mat::zeros(img_1.rows, img_1.cols, img_1.type());

    std::vector<cv::Point2f> yourPoints1;
    std::vector<cv::Point2f> yourPoints2;
    cv::Mat pt_img1 = cv::Mat::zeros(cv::Size(img_1.cols, img_1.rows), CV_8UC3);
    cv::Mat pt_img2 = cv::Mat::zeros(cv::Size(img_1.cols, img_1.rows), CV_8UC3);
    // yourPoints1.push_back(cv::Point2f(702,360));
    // yourPoints2.push_back(cv::Point2f(828,104));
    std::cout << "number of good matches: " << good_matches.size() << endl;

    // cv::circle(pt_img1,
    //         cv::Point(yourPoints1[0]), 5,
    //         CV_RGB(245, 23, 23), -1);

    // cv::circle(img_2,
    //         cv::Point(828, 104), 5,
    //         CV_RGB(245, 23, 23), -1);

    for(size_t i = 0; i < good_matches.size(); i++) {
        img1_pts[i] = Point2f(keypoints_1.at(good_matches[i].queryIdx).pt.x, keypoints_1.at(good_matches[i].queryIdx).pt.y);
        img2_pts[i] = Point2f(keypoints_2.at(good_matches[i].trainIdx).pt.x, keypoints_2.at(good_matches[i].trainIdx).pt.y);
        yourPoints1.push_back(img1_pts[i]);
        yourPoints2.push_back(img2_pts[i]);
        cout << "img1_pts: " << img1_pts[i] << endl;
        cout << "img2_pts: " << img2_pts[i] << endl;
        cv::circle(img_1,
                    cv::Point(img1_pts[i]), 3,
                    CV_RGB(245, 23, 23), -1);

        // cv::circle(img_2,
        //             cv::Point(img2_pts[i]), 3,
        //             CV_RGB(245, 23, 23), -1);
    }

    // lambda = getPerspectiveTransform(img1_pts, img2_pts);
    // lambda = getAffineTransform(img1_pts, img2_pts);
    lambda = findHomography(yourPoints1, yourPoints2, RANSAC, 9.0);
    Mat dst_mat = Mat::zeros(img_1.rows, img_1.cols, img_1.type());
    cout << "lambda = " << endl << " " << lambda << endl << endl;

    for (size_t i = 0; i < yourPoints1.size(); i++) {
        cv::Mat_<double> src(3/*rows*/,1 /* cols */); 

        src(0,0)=yourPoints1[i].x; 
        src(1,0)=yourPoints1[i].y; 
        src(2,0)=1.0; 

        cv::Mat_<double> dst = lambda*src; //USE MATRIX ALGEBRA
        // yourPoints1[i] = cv::Point2f(dst(0,0), dst(1,0));

        std::cout << "input" << i << " : " << yourPoints1[i] << std::endl;
        std::cout << "output" << i << ": " << cv::Point2f(dst(0,0), dst(1,0)) << std::endl;

        cv::circle(img_2,
                    cv::Point(cv::Point2f(dst(0,0), dst(1,0))), 3,
                    CV_RGB(245, 23, 23), -1);
    }
    
    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );

    
    
    // cv::circle(pt_img1,
    //                 cv::Point(702, 360), 5,
    //                 CV_RGB(245, 23, 23), -1);

   

    // std::vector<cv::Point2f> transformedPoints;

    // cv::perspectiveTransform(yourPoints1, transformedPoints, lambda);
    // cv::warpAffine(pt_img1, pt_img2, lambda, pt_img2.size());

    // for(size_t i = 0; i < transformedPoints.size(); i++) {
    //     std::cout << "point: " << transformedPoints[i] << endl;
    //     cv::circle(pt_img2,
    //                 cv::Point(transformedPoints[i]), 5,
    //                 CV_RGB(245, 66, 66), -1);
    // }

    // cv::circle(pt_img2,
    //                 cv::Point(transformedPoints[0]), 5,
    //                 CV_RGB(245, 23, 23), -1);

    // addWeighted(img_2, 1.0, pt_img2, 1.0, 0.0, img_2);
    // addWeighted(img_1, 1.0, pt_img1, 1.0, 0.0, img_1);

    // uint8_t* pixelPtr = (uint8_t*)img_match.data;
    // int cn = img_match.channels();
    // Scalar_<uint8_t> bgrPixel;

    // for(int i = 0; i < img_match.rows; i++)
    // {
    //     for(int j = 0; j < img_match.cols; j++)
    //     {
    //         if ((int)pixelPtr[i*img_match.cols*cn + j*cn + 0] == 0 && (int)pixelPtr[i*img_match.cols*cn + j*cn + 1] == 0 && (int)pixelPtr[i*img_match.cols*cn + j*cn + 2])
    //             cout << "pixel is black" << endl;
    //         else
    //             cout << "pixel is coloured" << endl;
    //     }
    // }
 
    // imshow ( "img_match", img_match );
    cv::resize(img_1, img_1, cv::Size(), 0.75, 0.75);
    cv::resize(img_2, img_2, cv::Size(), 0.75, 0.75);
    imshow ( "img_1", img_1 );
    imshow ( "img_2", img_2 );
    // imshow ( "img_goodmatch", img_goodmatch );
    waitKey(0);

    // double alpha = 0.5; double beta; double input;
    // Mat src1, src2, dst;
    // cout << " Simple Linear Blender " << endl;
    // cout << "-----------------------" << endl;
    // cout << "* Enter alpha [0.0-1.0]: ";
    // cin >> input;
    // // We use the alpha provided by the user if it is between 0 and 1
    // if( input >= 0 && input <= 1 )
    //     { alpha = input; }
    // src1 = imread( samples::findFile("/home/dd/calibration_ws/src/cam_radar_calibration/src/LinuxLogo.jpg") );
    // src2 = imread( samples::findFile("/home/dd/calibration_ws/src/cam_radar_calibration/src/WindowsLogo.jpg") );
    // if( src1.empty() ) { cout << "Error loading src1" << endl; return EXIT_FAILURE; }
    // if( src2.empty() ) { cout << "Error loading src2" << endl; return EXIT_FAILURE; }
    // beta = 1.0; //( 1.0 - alpha );
    // addWeighted( src1, alpha, src2, beta, 0.0, dst);
    // imshow( "Linear Blend", dst );
    // waitKey(0);

    return 0;
}