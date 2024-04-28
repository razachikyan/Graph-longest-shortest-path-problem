#include "./Models/Graph/Graph.hpp"
#include "./Views/MainWindow/MainWindow.hpp"

#include <QApplication>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    return app.exec();
}
