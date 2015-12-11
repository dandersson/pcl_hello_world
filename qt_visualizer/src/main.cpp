#include "pclviewer.h"

#include <QApplication>
#include <QMainWindow>

int main(int argc, char *argv[])
{
    if (argc > 2) {
        std::cerr <<
            "usage: " << argv[0] << " [PATH_TO_PCD_DIRECTORY]" << std::endl;
        exit(1);
    }

    std::string dir = (argc == 2 ? argv[1] : "");

    QApplication a(argc, argv);
    PCLViewer w(dir);
    w.show();
    return a.exec();
}
