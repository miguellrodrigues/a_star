#include <iostream>
#include <vector>
#include <algorithm>

#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "Cell.h"

using namespace cv;
using std::vector;

bool isDiagonal(Cell *c1, Cell *c2) {
    return c1->getX() != c2->getX() && c1->getY() != c2->getY();
}

std::vector<Cell *> getNeighbors(
        Cell *cell,
        const vector<vector<Cell *>> &cells,
        int width,
        int height
) {
    std::vector<Cell *> neighbors;
    neighbors.reserve(9);

    int i = cell->getI();
    int j = cell->getJ();

    if (i - 1 >= 0) {
        neighbors.emplace_back(cells.at(i - 1).at(j));
    }
    if (i + 1 < width) {
        neighbors.emplace_back(cells.at(i + 1).at(j));
    }

    if (j - 1 >= 0) {
        neighbors.emplace_back(cells.at(i).at(j - 1));
    }
    if (j + 1 < height) {
        neighbors.emplace_back(cells.at(i).at(j + 1));
    }

    // Diagonal neighbors

    if (i - 1 >= 0 && j - 1 >= 0) {
        neighbors.emplace_back(cells.at(i - 1).at(j - 1));
    }
    if (i + 1 < width && j - 1 >= 0) {
        neighbors.emplace_back(cells.at(i + 1).at(j - 1));
    }
    if (i - 1 >= 0 && j + 1 < height) {
        neighbors.emplace_back(cells.at(i - 1).at(j + 1));
    }
    if (i + 1 < width && j + 1 < height) {
        neighbors.emplace_back(cells.at(i + 1).at(j + 1));
    }

    return neighbors;
}

std::vector<Cell *> a_star(
        const vector<vector<Cell *>> &cells,
        Cell *start,
        Cell *end,
        int width,
        int height,
        int cellSize,
        Mat &img
) {

    double costDiagonal = sqrt((cellSize * cellSize) * 2);

    std::vector<Cell *> to_process;
    std::vector<Cell *> processed;

    to_process.push_back(start);

    while (!to_process.empty()) {
        // sort to_process by f_score
        std::sort(to_process.begin(), to_process.end(),
                  [](Cell *a, Cell *b) {
                      return a->getF() < b->getF();
                  }
        );

        Cell *current = to_process[0];

        to_process.erase(to_process.begin());

        cv::rectangle(img, cv::Point(current->getY1(), current->getX1()),
                      cv::Point(current->getY2(), current->getX2()), cv::Scalar(100, 150, 0), -1);

        processed.push_back(current);

        if (current == end || find(processed.begin(), processed.end(), end) != processed.end()) {
            break;
        }

        std::vector<Cell *> neighbors = getNeighbors(current, cells, width, height);

        // for each neighbour
        for (Cell *neighbor: neighbors) {
            if (neighbor->isObstacle()) {
                continue;
            }

            if (find(processed.begin(), processed.end(), neighbor) != processed.end())
                continue;

            // if neighbour is in to_process
            auto it = std::find(to_process.begin(), to_process.end(), neighbor);
            if (it != to_process.end()) {
                double g;

                if (isDiagonal(current, neighbor)) {
                    g = current->getG() + costDiagonal;
                } else {
                    g = current->getG() + cellSize;
                }

                if (g < neighbor->getG()) {
                    neighbor->setG(g);
                    neighbor->setF(g + neighbor->getH());
                    neighbor->setParent(current);
                }
            } else {
                neighbor->setParent(current);

                double h = sqrt(pow(neighbor->getX() - end->getX(), 2) + pow(neighbor->getY() - end->getY(), 2));

                if (isDiagonal(current, neighbor)) {
                    neighbor->setG(current->getG() + costDiagonal);
                } else {
                    neighbor->setG(current->getG() + cellSize);
                }

                neighbor->setH(h);
                neighbor->setF(neighbor->getG() + h);

                to_process.push_back(neighbor);

                // fill the cell bounding box
                cv::rectangle(img, cv::Point(neighbor->getY1(), neighbor->getX1()),
                              cv::Point(neighbor->getY2(), neighbor->getX2()), cv::Scalar(0, 255, 0), -1);
            }
        }

        neighbors.clear();

        cv::imshow("image", img);
        cv::waitKey(1);
    }

    to_process.clear();

    return processed;
}

void cellIndexFromPointOf(int x, int y, int cellSize, int &i, int &j) {
    i = y / cellSize;
    j = x / cellSize;
}

int main() {
    // load the image

    Mat lab = imread("/home/miguel/CLionProjects/astar/maze.png");

    if (lab.empty()) {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    Mat hsv, mask;
    cvtColor(lab, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(0, 0, 0), Scalar(180, 255, 150), mask);

    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    drawContours(lab, contours, -1, Scalar(255, 122, 231), 2);

    int width  = lab.rows;
    int height = lab.cols;

    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);

    int cell_size = 8; // pixels

    vector<vector<Cell *>> cells;
    cells.reserve((int) width/cell_size);

    int i = 0;
    int j = 0;

    for (int x = 0; x < width - cell_size; x += cell_size) {
        vector<Cell *> row;
        row.reserve((int) height/cell_size);

        for (int y = 0; y < height - cell_size; y += cell_size) {
            // the center of the cell is the center pixel between the pixels
            Cell *cell = new Cell((int) ((x + cell_size) / 2), (int) ((y + cell_size) / 2));
            cell->setBounds(x, y, x + cell_size, y + cell_size);

            cell->setIndex(i, j);

            j += 1;

            for (int k = x; k < x + cell_size; ++k) {
                for (int l = y; l < y + cell_size; ++l) {
                    auto pixel = mask.at<uchar>(k, l);

                    if (pixel == 255) {
                        cell->setObstacle(true);
                        //cv::rectangle(lab, cv::Point(y, x), cv::Point(y + cell_size, x + cell_size), cv::Scalar(255, 0, 255), 1);
                        break;
                    }
                }
            }

            row.push_back(cell);
        }

        i += 1;
        j          = 0;

        cells.push_back(row);
    }

    int start_i = 0;
    int start_j = 0;

    cellIndexFromPointOf(1240, 270, cell_size, start_i, start_j);

    int end_i = 0;
    int end_j = 0;

    cellIndexFromPointOf(945, 744, cell_size, end_i, end_j);

    Cell *start = cells[start_i][start_j];
    Cell *end   = cells[end_i][end_j];

    // find the path
    vector<Cell *> processed = a_star(cells, start, end, (int) cells.size(), (int) cells[0].size(), cell_size, lab);

    if (std::find(processed.begin(), processed.end(), end) == processed.end()) {
        std::cout << "No path found" << '\n';
        return 1;
    }

    vector<Cell *> path;
    path.push_back(start);

    Cell *current = end;

    while (current != start) {
        path.push_back(current);
        current = current->getParent();
    }

    // draw path
    for (Cell *cell: path) {
        cv::rectangle(lab, cv::Point(cell->getY1(), cell->getX1()), cv::Point(cell->getY2(), cell->getX2()),
                      cv::Scalar(0, 0, 255), -1);

        imshow("image", lab);
        waitKey(1);
    }

    waitKey(0);

    return 0;
}