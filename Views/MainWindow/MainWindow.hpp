#pragma once

#include "../../libs.hpp"
#include "./../GraphWidget/GraphWidget.hpp"

#include <QMainWindow>
#include <QLineEdit>
#include <QComboBox>
#include <QWidget>

class MainWindow : public QMainWindow {
public:
    MainWindow(QWidget *parent = nullptr);

private:
    void setupUI();
    void setupRightWidget();

private slots:
    void selectFile();
    void selectProblem(int index);
    void run();
    void selectStrategy(int index);

private:
    QWidget *centralWidget;
    QWidget *rightWidget;
    GraphWidget *graphWidget;
    QLineEdit *start;
    QLineEdit *end;
    QComboBox *strategies;
    QString selectedfilePath;
    std::string strSrategy;
};
