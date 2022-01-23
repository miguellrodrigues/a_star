//
// Created by miguel on 22/01/2022.
//

#ifndef ASTAR_CELL_H
#define ASTAR_CELL_H


class Cell {
public:
    Cell(int x, int y);

    int getX() const;
    int getY() const;

    int getI() const;
    int getJ() const;

    void setBounds(int a, int b, int c, int d);

    double getH() const;
    void setH(double _h);

    double getG() const;
    void setG(double _g);

    double getF() const;
    void setF(double _f);

    int getX1() const;
    int getY1() const;

    int getX2() const;
    int getY2() const;

    bool isObstacle() const;
    void setObstacle(bool o);

    Cell *getParent();
    void setParent(Cell *p);

    void setIndex(int i, int j);

private:
    int x;
    int y;

    int x1;
    int y1;

    int x2;
    int y2;

    int i{};
    int j{};

    double h{};
    double f{};
    double g{};

    bool obstacle{};
    Cell *parent{};
};


#endif //ASTAR_CELL_H
