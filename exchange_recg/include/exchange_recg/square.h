//
// Created by bismarck on 11/13/22.
//

#ifndef RM2022ENGINEER_SQUARE_H
#define RM2022ENGINEER_SQUARE_H

#include "exchange_recg/edge.h"

class square {
private:
    double edgeAngle = 0;
public:
    triangle corners[4];
    double aero = 0;
    int aux = 0;
    std::vector<cv::Point2f> points;
    std::vector<cv::Point2i> points2draw;

    square(edge e1, edge e2);
    bool isParallel() const;
    double getEdgeRate();
    void sortByAux();     // 以下标为i的三角形开头，逆时针寻找边线

    void draw(cv::Mat& image);
    void drawR(cv::Mat& image);
};


#endif //RM2022ENGINEER_SQUARE_H
