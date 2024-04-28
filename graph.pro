TEMPLATE = app
QT += core gui widgets
QMAKE_CXXFLAGS += -std=c++17

INCLUDEPATH += /usr/include/graphviz
LIBS += -lgvc -lcgraph -lcdt

SOURCES += main.cpp \
            Models/Graph/Graph.cpp \
            Strategy/Strategy.cpp \
            Models/Node/Node.cpp \
            Views/GraphWidget/GraphWidget.cpp \
            Views/MainWindow/MainWindow.cpp
