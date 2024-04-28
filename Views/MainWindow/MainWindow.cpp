#include "./MainWindow.hpp"
#include "./../../Models/Graph/Graph.hpp"

#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QIntValidator>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("Main Window");
    resize(1000, 800);

    centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    graphWidget = new GraphWidget(this);
    rightWidget = new QWidget(centralWidget);
    strategies = new QComboBox;
    strategies->setDisabled(true);

    setupUI();
}

void MainWindow::setupUI() {
    QHBoxLayout *layout = new QHBoxLayout(centralWidget);

    setupRightWidget();

    layout->addWidget(graphWidget);
    layout->addWidget(rightWidget);
}

void MainWindow::setupRightWidget() {
    rightWidget->setMinimumWidth(300);

    QVBoxLayout *layout = new QVBoxLayout(rightWidget);
    layout->setAlignment(Qt::AlignTop);

    QPushButton *selectFileButton = new QPushButton("Select File");
    connect(selectFileButton, &QPushButton::clicked, this, &MainWindow::selectFile);

    QComboBox* problems = new QComboBox;
    problems->addItem("None");
    problems->addItem("Shortest path");
    problems->addItem("Lognest path");
    connect(problems, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::selectProblem);

    QPushButton *runButton = new QPushButton("Run");
    connect(runButton, &QPushButton::clicked, this, &MainWindow::run);

    start = new QLineEdit;
    start->setValidator(new QIntValidator(start));
    end = new QLineEdit;
    end->setValidator(new QIntValidator(end));

    connect(strategies, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::selectStrategy);

    layout->addWidget(runButton);
    layout->addWidget(selectFileButton);
    layout->addWidget(new QLabel("Select Problem:"));
    layout->addWidget(problems);
    layout->addWidget(new QLabel("Select Algorithm:"));
    layout->addWidget(strategies);
    layout->addWidget(new QLabel("Start:"));
    layout->addWidget(start);
    layout->addWidget(new QLabel("End:"));
    layout->addWidget(end);
}

void MainWindow::selectFile() {
    selectedfilePath = QFileDialog::getOpenFileName(this, tr("Select File"), "", tr("All Files (*)"));
}

void MainWindow::run() {
    if (selectedfilePath.isEmpty()) {
        QMessageBox::warning(this, tr("Warning"), tr("No file selected!"), QMessageBox::Ok);
        return;
    }
    if (strSrategy.empty()) {
        QMessageBox::warning(this, tr("Warning"), tr("No strategy selected!"), QMessageBox::Ok);
        return;
    }

    Graph graph;
    graph.readGraphFromFile(selectedfilePath.toStdString());

    QString strStart = start->text();
    QString strEnd = end->text();
    if(strStart.toInt() < 0 || strStart.toInt() >= graph.getMatrix().size()
    || strEnd.toInt() < 0 || strEnd.toInt() >= graph.getMatrix().size() 
    || strStart.isEmpty() || strEnd.isEmpty())
    {
        QMessageBox::warning(this, tr("Warning"), tr("Invalid targets!"), QMessageBox::Ok);
        return;
    }

    graph.setStrategy(strSrategy);
    graphWidget->drawGraph(graph.getMatrix(), graph.getShortestPath(strStart.toInt(), strEnd.toInt()));
}

void MainWindow::selectStrategy(int index) {
    strSrategy = strategies->itemText(index).toStdString();
}

void MainWindow::selectProblem(int index) {
    if(!strategies->isEnabled()) {
        strategies->setEnabled(true);
    }
    strategies->clear();
    if(index == 0) {   
        strategies->setDisabled(true);
    }

    if(index == 1) {
        strategies->addItem("AStar");
        strategies->addItem("Dijkstra");
        
        //Default algo
        strSrategy = "AStar";
    } else {
        strategies->addItem("BruteForce");
        strategies->addItem("Genetic");

        //Default algo
        strSrategy = "BruteForce";
    }
}