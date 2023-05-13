//
// Created by bismarck on 11/13/22.
//

#include "exchange_recg/utils.h"

double getAngleCosByPoints(cv::Point2d e1, cv::Point2d center, cv::Point2d e2) {
    cv::Point2d vec1 = e1 - center;
    cv::Point2d vec2 = e2 - center;
    double result = vec1.dot(vec2) / norm(vec1) / norm(vec2);
    return result;
}

angle_d getEdgeAngle(cv::Point2d p1, cv::Point2d p2) {
    cv::Point2d p = p1 - p2;
    return atan2(p.y, p.x);
}

double getLength(cv::Point2d p1, cv::Point2d p2) {
    return norm(p1 - p2);
}


