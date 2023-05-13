//
// Created by bismarck on 11/13/22.
//

#ifndef RM2022ENGINEER_MANAGER_H
#define RM2022ENGINEER_MANAGER_H

#include <vector>

#include <opencv4/opencv2/opencv.hpp>

class triangle;
class edge;
class square;
class auxPoint;

class manager {
private:
    std::vector<triangle> triangles;
    std::vector<edge> edges;
    std::vector<square> squares;
    std::vector<auxPoint> auxPoints;
    int nowTid = 0;
    void addContoursAsAuxPoint(cv::InputArray contour);

public:
    bool addContours(cv::InputArray contour);
    int calculateEdges();
    int calculateSquares();
    int isSquareAux(int sid, cv::Point2d point, float size);

    void sortSquare();
    square getSquare(int sid);

    void draw(cv::Mat& image);
    bool empty();
};


#endif //RM2022ENGINEER_MANAGER_H
