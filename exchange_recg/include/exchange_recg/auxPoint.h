//
// Created by bismarck on 11/13/22.
//

#ifndef RM2022ENGINEER_AUXPOINT_H
#define RM2022ENGINEER_AUXPOINT_H

#include <opencv4/opencv2/opencv.hpp>

class auxPoint {
public:
    double aero;
    cv::Point2f center;
    float radius=0;
    bool match = false;

    auxPoint(cv::InputArray contour);

    void draw(cv::Mat& image);
};


#endif //RM2022ENGINEER_AUXPOINT_H
