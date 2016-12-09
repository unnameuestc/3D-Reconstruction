#include "mainwindow.h"
#include <QApplication>
#include <QDesktopWidget>
#include <QTextCodec>
#include <QTranslator>
#include <QtWidgets/QApplication>
#include <QDir>

#include "mymatcher.h"

int main(int argc, char *argv[])
{
    //return MyMatcher::test();

    QApplication app(argc, argv);

    //工作路径
    QDir::setCurrent(app.applicationDirPath());
    QApplication::addLibraryPath(QDir::currentPath());

    //编码
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));

    //全局样式
    QFile qssFile("://style.qss");
    qssFile.open(QFile::ReadOnly);
    if(qssFile.isOpen())
    {
        QString qss = qssFile.readAll();
        app.setStyleSheet(qss);
        qssFile.close();
    }

    MainWindow w;
    w.show();

    return app.exec();
}
