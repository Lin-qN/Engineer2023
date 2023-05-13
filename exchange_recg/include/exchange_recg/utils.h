//
// Created by bismarck on 11/13/22.
//

#ifndef RM2022ENGINEER_UTILS_H
#define RM2022ENGINEER_UTILS_H

#include <cmath>

#include <opencv4/opencv2/opencv.hpp>

template<typename T>
class angle {
    T num;
public:
    angle(T t) {num = t;}
    angle operator-(const angle<T> a1) {
        T n = abs(num - a1.num);
        while (n > 2 * M_PIl) {
            n -= 2 * M_PIl;
        }
        if (n > M_PIl) {
            n = 2 * M_PIl - n;
        }
        if (n > M_PI_2l) {
            n = M_PIl - n;
        }
        return n;
    }
    operator T() const {
        return num;
    }
    angle<T> reverse() {
        num += M_PIl;
        return *this;
    }
};

typedef angle<float> angle_f;
typedef angle<double> angle_d;

double getAngleCosByPoints(cv::Point2d e1, cv::Point2d center, cv::Point2d e2);
angle_d getEdgeAngle(cv::Point2d p1, cv::Point2d p2);
double getLength(cv::Point2d p1, cv::Point2d p2);

constexpr double degree2rad(double degree) {
    return degree / 180 * M_PIl;
}

#endif //RM2022ENGINEER_UTILS_H
