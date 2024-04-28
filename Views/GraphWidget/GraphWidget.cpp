#include "./GraphWidget.hpp"

GraphWidget::GraphWidget(QWidget *parent) : QWidget(parent) {
    QVBoxLayout *layout = new QVBoxLayout(this);
    scene_ = new QGraphicsScene(this);
    view = new QGraphicsView(scene_);
    layout->addWidget(view);
}

void GraphWidget::drawGraph(const std::vector<std::vector<int>>& adjacencyMatrix, std::vector<int> path) {
    int numNodes = adjacencyMatrix.size();

    QString dotGraph = "graph G {\n";
    for (int i = 0; i < numNodes; ++i) {
        for (int j = i; j < numNodes; ++j) {
            if (adjacencyMatrix[i][j] > 0) {
                QString styles;
                auto first = std::find(path.begin(), path.end(), i);
                auto second = std::find(path.begin(), path.end(), j);
                if(first != path.end() && second != path.end() && (std::next(first) == second || std::next(second) == first))
                {
                    styles = "[label=\"%3\", color=\"red\"]";
                }
                else
                {
                    styles = "[label=\"%3\"]";
                }
                dotGraph += QString("    \"%1\" -- \"%2\" " + styles + ";\n")
                                .arg(QString::number(i))
                                .arg(QString::number(j))
                                .arg(QString::number(adjacencyMatrix[i][j]));
            }
        }
    }
    dotGraph += "}\n";

    QProcess process;
    process.setProcessChannelMode(QProcess::MergedChannels);
    process.start("dot", QStringList() << "-Tpng");

    process.write(dotGraph.toUtf8());
    process.closeWriteChannel();

    process.waitForFinished();

    QByteArray imageData = process.readAll();

    QPixmap pixmap;
    pixmap.loadFromData(imageData);
    scene_->addPixmap(pixmap);
}
