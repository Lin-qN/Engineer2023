//
// Created by bismarck on 11/13/22.
//

#ifndef RM2022ENGINEER_TRIANGLE_H
#define RM2022ENGINEER_TRIANGLE_H

#include <opencv4/opencv2/opencv.hpp>

#include "exchange_recg/auxPoint.h"
#include "exchange_recg/utils.h"

class triangle {
public:
    double aero = 0;
    cv::Point2d points[3];
    double angle_cos = 0;
    angle_d edge_angle[2]{0, 0};
    int index = 0;
    int aux = 0;

    triangle() = default;
    triangle(cv::InputArray contour);

    bool isCorner();
    bool hasAux(auxPoint p);

    void draw(cv::Mat& image);
};


#endif //RM2022ENGINEER_TRIANGLE_H
