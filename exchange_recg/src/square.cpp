//
// Created by bismarck on 11/13/22.
//

#include "exchange_recg/config.h"

#include "exchange_recg/square.h"

square::square(edge e1, edge e2) {
    edgeAngle = e1.edge_angle - e2.edge_angle;
    corners[0] = e1.corners[0];
    corners[1] = e1.corners[1];
    corners[2] = e2.corners[0];
    corners[3] = e2.corners[1];

    std::vector<cv::Point> temp;

    for (auto & corner : corners) {
        temp.push_back(corner.points[1]);
    }
    
    aero = cv::contourArea(temp);
}

bool square::isParallel() const {
    return edgeAngle < MAX_EDGE_ANGLE_DIFF;
}

bool cmp_y(triangle& A, triangle& B)
{
    return A.points[1].y > B.points[1].y;
}

void swap(triangle& A, triangle& B) {
    triangle C = A;
    A = B;
    B = C;
}

void square::sortByAux() {
    std::sort(corners, corners + 4, cmp_y);
    if(corners[1].points[1].x < corners[0].points[1].x)
        swap(corners[0], corners[1]);
    if(corners[2].points[1].x < corners[3].points[1].x)
        swap(corners[2], corners[3]);

    int maxAux = 0;
    int maxAuxIndex = 0;
    for (int i = 0; i < 4; i++) {
        if (corners[i].aux > maxAux) {
            maxAux = corners[i].aux;
            maxAuxIndex = i;
        }
    }
    aux = maxAux;
    maxAuxIndex = 2;

    points.push_back(corners[(maxAuxIndex+2) % 4].points[1]);
    points.push_back(corners[(maxAuxIndex+3) % 4].points[1]);
    points.push_back(corners[maxAuxIndex].points[1]);
    points.push_back(corners[(maxAuxIndex+1) % 4].points[1]);

    points2draw.push_back(corners[(maxAuxIndex+2) % 4].points[1]);
    points2draw.push_back(corners[(maxAuxIndex+3) % 4].points[1]);
    points2draw.push_back(corners[maxAuxIndex].points[1]);
    points2draw.push_back(corners[(maxAuxIndex+1) % 4].points[1]);
}

void square::draw(cv::Mat& image) {
    circle(image, points[0], 3, cv::Scalar(200, 20, 20), 2);
    circle(image, points[1], 3, cv::Scalar(20, 200, 20), 2);
    circle(image, points[2], 3, cv::Scalar(200, 20, 200), 2);
    circle(image, points[3], 3, cv::Scalar(200, 200, 20), 2);
}

void square::drawR(cv::Mat &image) {
    cv::polylines(image, points2draw, true, cv::Scalar(255, 0, 0), 4);
}

double square::getEdgeRate() {
    double min = 1e20, max = 0;
    for (int i = 0; i < 4; i++) {
        auto pd = points[i] - points[(i + 1) % 4];
        double len = sqrt(pow(pd.x, 2) + pow(pd.y, 2));
        if (len > max) {
            max = len;
        }
        if (len < min) {
            min = len;
        }
    }
    return min / max;
}
