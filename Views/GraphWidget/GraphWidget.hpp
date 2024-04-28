#pragma once

#include "../../libs.hpp"

#include <QMainWindow>
#include <QVBoxLayout>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QProcess>

class GraphWidget : public QWidget {
public:
    GraphWidget(QWidget *parent = nullptr);
    void drawGraph(const std::vector<std::vector<int>>& adjacencyMatrix, std::vector<int> path);

private:
    QGraphicsScene *scene_;
    QGraphicsView *view;
};
