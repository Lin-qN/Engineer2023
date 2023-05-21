//
// Created by catmulti7 on 22-7-25.
//
#include <vector>

#include <ros/ros.h>
#include <opencv4/opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "exchange_recg/manager.h"

#include "exchange_recg/square.h"
using namespace std;

using namespace cv;
ros::Subscriber imageSub;
ros::Subscriber modeSub;
ros::Publisher pointPub;
ros::Publisher imagePub;

ros::Publisher binPub;
Mat img, img2Show, imgBin;
bool ifRed, ifShow, ifPub;
int thresh;
Mat cameraMatrix, distCoeffs;
vector<Point3d> realPoints;
Point2d offset;
::uint8_t mode;

const double PI =3.14159;
static void toEulerAngle(const double x,const double y,const double z,const double w, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
    //    return yaw;
}

void addImageOffset(vector<Point2d> &points) {
    for (auto &p: points) {
        p = p + offset;
    }
}

int bgr2binary(Mat &srcImg, Mat &img_out, int method, int thresh, bool if_Red) {
    if (srcImg.empty())
        return -1;
    if (method == 1) {
        //method 1: split channels and substract
        vector<Mat> imgChannels;
        split(srcImg, imgChannels);
        Mat red_channel = imgChannels.at(2);
        Mat green_channel = imgChannels.at(1);
        Mat blue_channel = imgChannels.at(0);
        Mat mid_chn_img;

        if (if_Red) {
            mid_chn_img = red_channel - blue_channel - green_channel;

        } else {
            mid_chn_img = blue_channel - red_channel - green_channel;
        }

        threshold(mid_chn_img, img_out, thresh, 255, THRESH_BINARY);
    }
    if (method == 2) {
        //method 2:
        Mat imgGray;
        cvtColor(srcImg, imgGray, COLOR_BGR2GRAY);
        threshold(imgGray, img_out, thresh, 255, THRESH_BINARY);
    }
    return 0;
}

void getParams(ros::NodeHandle &nh) {
    nh.getParam("/if_red", ifRed);
    nh.getParam("/if_show", ifShow);
    nh.getParam("/if_pub", ifPub);
    nh.getParam("/threshold", thresh);
    double fx, fy, cx, cy, distcoef1, distcoef2;
    nh.getParam("/CameraParams/cx", cx);
    nh.getParam("/CameraParams/cy", cy);
    nh.getParam("/CameraParams/fx", fx);
    nh.getParam("/CameraParams/fy", fy);
    nh.getParam("/CameraParams/distcoef1", distcoef1);
    nh.getParam("/CameraParams/distcoef2", distcoef2);
    cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    distCoeffs = (Mat_<double>(1, 4) << distcoef1, distcoef2, 0, 0);

    realPoints = {Point3d(-0.144, 0.144, 0), Point3d(0.144, 0.144, 0),
                  Point3d(0.144, -0.144, 0), Point3d(-0.144, -0.144, 0)};

    float offsetH, offsetW;
    nh.getParam("/offset_H", offsetH);
    nh.getParam("/offset_W", offsetW);
    offset = Point2f(offsetW, offsetH);
}

inline void getQuaternion(Mat R, double Q[]) {
    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);

    if (trace > 0.0) {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2, 1) - R.at<double>(1, 2)) * s);
        Q[1] = ((R.at<double>(0, 2) - R.at<double>(2, 0)) * s);
        Q[2] = ((R.at<double>(1, 0) - R.at<double>(0, 1)) * s);
    } else {
        int i = R.at<double>(0, 0) < R.at<double>(1, 1) ? (R.at<double>(1, 1) < R.at<double>(2, 2) ? 2 : 1) : (
                R.at<double>(0, 0) < R.at<double>(2, 2) ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j, j) - R.at<double>(k, k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<double>(k, j) - R.at<double>(j, k)) * s;
        Q[j] = (R.at<double>(j, i) + R.at<double>(i, j)) * s;
        Q[k] = (R.at<double>(k, i) + R.at<double>(i, k)) * s;
    }
}

void getExchangePose(const sensor_msgs::ImageConstPtr &msg){
    //convert ROS image msg to opencv Mat
    try {
        img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        // markSensor->imgTime = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->header.stamp;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (img.empty())
        return;

    if (ifShow || ifPub)
    {
        img2Show = img.clone();
        //img2Show = img2Show/1.5;
    }


    bgr2binary(img2Show, imgBin, 1, thresh, ifRed);

    if (ifShow) {
        //imshow("threshold", imgBin);
        sensor_msgs::ImagePtr _msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgBin).toImageMsg();
        binPub.publish(_msg);
    }
    waitKey(1);

    vector<vector<Point>> contours;
    findContours(imgBin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    manager man;
    for (const auto &contour: contours) {
        man.addContours(contour);
    }

    try {
        man.calculateEdges();
        man.calculateSquares();
        man.sortSquare();
    } catch (cv::Exception &e) {
        ROS_ERROR("opencv exception: %s", e.what());
        return;
    }

    double theta, _sin, roll, pitch, yaw;
    Eigen::Vector3d euler;
    if (!man.empty()) {
        square s = man.getSquare(0);
        cout << "aux num: " << s.aux << endl;
        vector<Point2f> sortedImagePoints = s.points;
        Mat rvec, tvec(3, 1, CV_32FC1);
        Eigen::Quaterniond q;

        try {
            Mat _tvec;
            solvePnP(realPoints, sortedImagePoints, cameraMatrix, distCoeffs, rvec, _tvec, false, SOLVEPNP_IPPE_SQUARE);
            if (_tvec.empty()) {
                return;
            }
            tvec.at<float>(0, 0) = (float)_tvec.at<double>(2, 0);
            tvec.at<float>(1, 0) = -(float)_tvec.at<double>(0, 0);
            tvec.at<float>(2, 0) = -(float)_tvec.at<double>(1, 0);

            cout << rvec << endl;
            theta = cv::norm(rvec);

            _sin = sin(theta * 0.5) / theta;
            q.w() = cos(theta * 0.5);
            q.x() = rvec.at<double>(0, 2) * _sin;
            q.y() = -rvec.at<double>(0, 0) * _sin;
            q.z() = -rvec.at<double>(0, 1) * _sin;
            euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
            cout << "Quaterniond2Euler result is:" <<endl;
            cout << "x = "<< euler[2]*180/PI << endl ;
            cout << "y = "<< euler[1]*180/PI << endl ;
            cout << "z = "<< euler[0]*180/PI << endl << endl;

//            toEulerAngle(q.x(),q.y(),q.z(),q.w(),roll,pitch,yaw);
//            cout<<roll*180/PI<<endl<<pitch*180/PI<<endl<<yaw*180/PI<<endl;

        } catch (cv::Exception &e) {
            ROS_ERROR("opencv exception: %s", e.what());
            return;
        }

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = tvec.at<float>(0, 0);
        pose.pose.position.y = tvec.at<float>(1, 0);
        pose.pose.position.z = tvec.at<float>(2, 0);
        pose.pose.orientation.x = euler[2]*180/PI;
        pose.pose.orientation.y = euler[1]*180/PI;
        pose.pose.orientation.z = euler[0]*180/PI;
        pose.pose.orientation.w = 0;
        pointPub.publish(pose);

//    if (ifShow || ifPub) {
//        try {
        man.draw(img2Show);
    }


//            if (ifShow) {
                //imshow("img2Show", img2Show);
                //waitKey(1);
//            }
//            if (ifPub) {
                sensor_msgs::ImagePtr _msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img2Show).toImageMsg();
                imagePub.publish(_msg);
//            }
//        } catch (cv::Exception &e) {
//            ROS_ERROR("opencv exception: %s", e.what());
//            return;
//        }
//    }
}

void imgCB(const sensor_msgs::ImageConstPtr &msg) {
    //if (mode == 0x11) {
        getExchangePose(msg);
    //}
}

void modeCB(const std_msgs::UInt8ConstPtr &msg) {
    mode = msg->data;
}

int main(int argc, char **argv) {
    //初始化节点
    ros::init(argc, argv, "recg_node");
    //声明节点句柄
    ros::NodeHandle nh;

    getParams(nh);

    imageSub = nh.subscribe<sensor_msgs::Image>("/raw_img", 1, imgCB);
    modeSub = nh.subscribe<std_msgs::UInt8>("/robot/mode", 1, modeCB);
    pointPub = nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);
    imagePub = nh.advertise<sensor_msgs::Image>("/proc_img", 1);
    binPub = nh.advertise<sensor_msgs::Image>("/bin_img", 1);
    ros::spin();
}