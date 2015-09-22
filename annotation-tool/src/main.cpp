#include <QtGui/QApplication>
#include "mainwindow.h"


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow w;

    // Show the window occupying the whole screen
    w.showMaximized();
    w.showInitialMessage();


    return app.exec();
}
