//
// Created by bismarck on 11/13/22.
//

#ifndef RM2022ENGINEER_EDGE_H
#define RM2022ENGINEER_EDGE_H

#include "exchange_recg/triangle.h"

class edge {
public:
    triangle corners[2];
    int pointIndex[2]{0};   // 构成边的点在triangle中的index
    angle_d edge_angle{0};

    edge() = default;
    edge(triangle t1, triangle t2);
    bool isEdge();      // 只有在isEdge执行后才会将所有的成员都初始化
    bool isInEdge(int tid);

    void draw(cv::Mat& image);
};


#endif //RM2022ENGINEER_EDGE_H
