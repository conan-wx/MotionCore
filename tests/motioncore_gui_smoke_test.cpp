#include "mainwindow.hpp"

#include <QApplication>
#include <QTimer>

int main(int argc, char** argv) {
    QApplication application(argc, argv);
    MainWindow window;
    window.show();
    QTimer::singleShot(250, &application, &QCoreApplication::quit);
    return application.exec();
}

