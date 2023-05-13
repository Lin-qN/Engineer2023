//
// Created by bismarck on 11/13/22.
//
#include "exchange_recg/auxPoint.h"

auxPoint::auxPoint(const cv::_InputArray &contour) {
    aero = cv::contourArea(contour);
    minEnclosingCircle(contour, center, radius);
}

void auxPoint::draw(cv::Mat& image) {
    circle(image, center, radius, cv::Scalar(0, 255, 0), 3);
}
