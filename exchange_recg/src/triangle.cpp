//
// Created by bismarck on 11/13/22.
//

#include "exchange_recg/config.h"

#include "exchange_recg/triangle.h"

triangle::triangle(cv::InputArray contour) {
    std::vector<cv::Point2d> _triangle;
    aero = cv::contourArea(contour);
    cv::minEnclosingTriangle(contour, _triangle);
    double minAngleCos = 1;      // [0~180]度范围内cos函数单调递减，故最小cos为最大角
    int minIndex = 0;
    for (int i = 0; i < 3; i++) {
        double angleCos = getAngleCosByPoints(_triangle[(i + 1) % 3], _triangle[i], _triangle[(i + 2) % 3]);
        if (angleCos < minAngleCos) {
            minAngleCos = angleCos;
            minIndex = i;
        }
    }
    points[0] = _triangle[(minIndex+1)%3];
    points[1] = _triangle[minIndex];
    points[2] = _triangle[(minIndex+2)%3];
    angle_cos = minAngleCos;
    edge_angle[0] = getEdgeAngle(points[1], points[0]);
    edge_angle[1] = getEdgeAngle(points[1], points[2]);
}

bool triangle::isCorner() {
    if (aero < MIN_AERO) {
        return false;
    }
    if (angle_cos > cos(degree2rad(MIN_ANGLE))) {
        return false;
    }
    double l1 = getLength(points[0], points[1]);
    double l2 = getLength(points[0], points[2]);
    double diff = abs(l1 - l2) / std::min(l1, l2);
    if (diff > MAX_LENGTH_DIFF) {
        return false;
    }
    return true;
}

void triangle::draw(cv::Mat &image) {
    for(int i = 0; i < 3; i++) {
        cv::line(image, points[i], points[(i+1)%3], cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
        circle(image, points[1], 3, cv::Scalar(200, 20, 20), 2);
    }
}

bool triangle::hasAux(auxPoint p) {
    double aeroRate = p.aero / aero;
    if (aeroRate < AUX_MIN_AERO || aeroRate > AUX_MAX_AERO) {
        return false;
    }
    bool has = false;
    for (auto & i : edge_angle) {
        angle_d toAux = getEdgeAngle(points[1], p.center);
        if (toAux - i > degree2rad(AUX_MAX_ANGLE)) {
            continue;
        }
        double length = getLength(points[1], p.center) / sqrt(p.aero);
        if (length < AUX_MIN_LENGTH || length > AUX_MAX_LENGTH) {
            continue;
        }
        has = true;
    }
    return has;
}
