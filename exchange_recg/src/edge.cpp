//
// Created by bismarck on 11/13/22.
//

#include "exchange_recg/config.h"

#include "exchange_recg/edge.h"

edge::edge(triangle t1, triangle t2) {
    corners[0] = t1;
    corners[1] = t2;
}

bool edge::isEdge() {
    if (abs(corners[0].aero - corners[1].aero) / std::min(corners[0].aero, corners[1].aero) > MAX_AERO_DIFF) {
        return false;
    }
    double minDiff = 3.1 * M_PIl;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            /*
            cv::Mat image = cv::Mat::zeros(1080, 1920, CV_8UC3);
            corners[0].draw(image);
            corners[1].draw(image);
            cv::line(image, corners[0].points[1], corners[1].points[1], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            cv::line(image, corners[0].points[i*2], corners[0].points[1], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            cv::line(image, corners[1].points[j*2], corners[1].points[1], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            // DEBUG: 需要详细debug边的匹配时请解注释这段代码，然后启动调试，断点设置在本行，进入断点后查看image
            */
            angle_d tris_angle = corners[0].edge_angle[i] - corners[1].edge_angle[j].reverse();
            if (tris_angle > degree2rad(MAX_TRIANGLE_EDGE_ANGLE_DIFF)) {
                continue;
            }
            angle_d edgeAngle = getEdgeAngle(corners[0].points[1], corners[1].points[1]);
            angle_d diff1 = edgeAngle - corners[0].edge_angle[i];
            if (diff1 > degree2rad(MAX_EDGE_TRIANGLE_DIFF)) {
                continue;
            }
            angle_d diff2 = edgeAngle - corners[1].edge_angle[j].reverse();
            if (diff2 > degree2rad(MAX_EDGE_TRIANGLE_DIFF)) {
                continue;
            }
            double totalDiff = diff1 + diff2 + tris_angle;
            if (totalDiff < minDiff) {
                edge_angle = edgeAngle;
                pointIndex[0] = i * 2;      // 将[0, 1]重映射为[0, 2]
                pointIndex[1] = j * 2;
                minDiff = totalDiff;
            }
        }
    }
    return minDiff < 3 * M_PIl;
}

bool edge::isInEdge(int tid) {
    return corners[0].index == tid || corners[1].index == tid;
}

void edge::draw(cv::Mat &image) {
    cv::line(image, corners[0].points[1], corners[1].points[1], cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
}
