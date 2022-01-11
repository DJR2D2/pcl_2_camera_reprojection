#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

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
    imshow("ORB特征点",outimg1);


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

    for(size_t i = 0; i < good_matches.size(); i++) {
        img1_pts[i] = Point2f(keypoints_1.at(good_matches[i].queryIdx).pt.x, keypoints_1.at(good_matches[i].queryIdx).pt.y);
        img2_pts[i] = Point2f(keypoints_2.at(good_matches[i].trainIdx).pt.x, keypoints_2.at(good_matches[i].trainIdx).pt.y);
    }

    lambda = getPerspectiveTransform(img1_pts, img2_pts);
    cout << "lambda = " << endl << " " << lambda << endl << endl;

    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );

    uint8_t* pixelPtr = (uint8_t*)img_match.data;
    int cn = img_match.channels();
    Scalar_<uint8_t> bgrPixel;

    for(int i = 0; i < img_match.rows; i++)
    {
        for(int j = 0; j < img_match.cols; j++)
        {
            if ((int)pixelPtr[i*img_match.cols*cn + j*cn + 0] == 0 && (int)pixelPtr[i*img_match.cols*cn + j*cn + 1] == 0 && (int)pixelPtr[i*img_match.cols*cn + j*cn + 2])
                cout << "pixel is black" << endl;
            else
                cout << "pixel is coloured" << endl;
        }
    }
    // for (int i = 0; i < img_match.rows; ++i) {
    //     uchar *row_ptr = img_match.ptr<uchar>(i);
    //     for (int j = 0; j < img_match.cols; ++j) {
    //         std::cout << "pixel value is " << img_match.at<Vec3b>(i, j).val[1] << std::endl;
    //     }
    // }
    imshow ( "img_match", img_match );
    imshow ( "img_goodmatch", img_goodmatch );
    waitKey(0);

    return 0;
}