//
// Created by bismarck on 11/13/22.
//

#include "exchange_recg/triangle.h"
#include "exchange_recg/edge.h"
#include "exchange_recg/square.h"
#include "exchange_recg/auxPoint.h"
#include "exchange_recg/config.h"

#include "exchange_recg/manager.h"

bool manager::addContours(cv::InputArray contour) {
    triangle tri = triangle(contour);
    if (tri.isCorner()) {
        tri.index = nowTid++;
        triangles.push_back(tri);
        addContoursAsAuxPoint(contour);
        return true;
    }
    addContoursAsAuxPoint(contour);
    return false;
}

int manager::calculateEdges() {
    if (triangles.size() < 2) {
        return 0;
    }
    for (int i = 0; i < triangles.size(); i++) {
        for (int j = 0; j < auxPoints.size(); j++) {
            if (auxPoints[j].match) {
                continue;
            }
            if (triangles[i].hasAux(auxPoints[j])) {
                auxPoints[j].match = true;
                triangles[i].aero += 2 * auxPoints[j].aero;
                triangles[i].aux++;
            }
        }
    }
    for (int i = 0; i < triangles.size() - 1; i++) {
        for (int j = i + 1; j < triangles.size(); j++) {
            edge e(triangles[i], triangles[j]);
            if (e.isEdge()) {
                edges.push_back(e);
            }
        }
    }
    return (int)edges.size();
}

int manager::calculateSquares() {
    if (edges.size() < 2) {
        return 0;
    }
    for (int i = 0; i < edges.size() - 1; i++) {
        for (int j = i + 1; j < edges.size(); j++) {
            square s(edges[i], edges[j]);
            if (s.isParallel()) {
                int ok[4] = {0};
                for (int k = 0; k < edges.size(); k++) {
                    if (k == i || k == j){
                        continue;
                    }
                    bool isIn[4];
                    isIn[0] = edges[k].isInEdge(edges[i].corners[0].index);
                    isIn[1] = edges[k].isInEdge(edges[i].corners[1].index);
                    isIn[2] = edges[k].isInEdge(edges[j].corners[0].index);
                    isIn[3] = edges[k].isInEdge(edges[j].corners[1].index);
                    for (int u = 0; u < 2; u++) {
                        for (int v = 2; v < 4; v++) {
                            if (isIn[u] && isIn[v]) {
                                ok[u]++;
                                ok[v]++;
                            }
                        }
                    }
                }
                if (ok[0] > 0 && ok[1] > 0 && ok[2] > 0 && ok[3] > 0){
                    s.sortByAux();
                    if (s.aux >= MIN_AUX_FOUND && s.aero > MIN_SQUARE_AERO && s.getEdgeRate() > MIN_EDGE_RATE) {
                        squares.push_back(s);
                    }
                }
            }
        }
    }
    return (int)squares.size();
}

int manager::isSquareAux(int sid, cv::Point2d point, float size) {
    return 0;
}

square manager::getSquare(int sid) {
    return squares[sid];
}

bool cmp_aero(const square& s1, const square& s2) {
    return s1.aero > s2.aero;
}

void manager::sortSquare() {
    sort(squares.begin(), squares.end(), cmp_aero);
}

void manager::draw(cv::Mat& image) {
    for (auto& auxPoint : auxPoints) {
        if (auxPoint.match) {
            auxPoint.draw(image);
        }
    }
    for (auto& triangle : triangles) {
        triangle.draw(image);
    }
    for (auto& edge : edges) {
        edge.draw(image);
    }
    for (auto& square : squares) {
        square.draw(image);
    }
    squares[0].drawR(image);
}

bool manager::empty() {
    return squares.empty();
}

void manager::addContoursAsAuxPoint(cv::InputArray contour) {
    auxPoint p(contour);
    if (p.aero > MIN_AUX_POINT_AERO) {
        auxPoints.push_back(p);
    }
}
