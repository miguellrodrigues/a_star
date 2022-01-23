//
// Created by miguel on 22/01/2022.
//

#include "Cell.h"

Cell::Cell(int x, int y) {
    this->x = x;
    this->y = y;
}

int Cell::getX() const {
    return x;
}

int Cell::getY() const {
    return y;
}

int Cell::getI() const {
    return i;
}

int Cell::getJ() const {
    return j;
}

void Cell::setBounds(int a, int b, int c, int d) {
    this->x1 = a;
    this->x2 = b;
    this->y1 = c;
    this->y2 = d;
}

double Cell::getH() const {
    return h;
}

void Cell::setH(double _h) {
    this->h = _h;
}

double Cell::getG() const {
    return g;
}

void Cell::setG(double _g) {
    this->g = _g;
}

double Cell::getF() const {
    return f;
}

void Cell::setF(double _f) {
    this->f = _f;
}

int Cell::getX1() const {
    return x1;
}

int Cell::getY1() const {
    return x2;
}

int Cell::getX2() const {
    return y1;
}

int Cell::getY2() const {
    return y2;
}

bool Cell::isObstacle() const {
    return obstacle;
}

void Cell::setObstacle(bool o) {
    this->obstacle = o;
}

Cell *Cell::getParent() {
    return parent;
}

void Cell::setParent(Cell *p) {
    this->parent = p;
}

void Cell::setIndex(int _i, int _j) {
    this->i = _i;
    this->j = _j;
}
