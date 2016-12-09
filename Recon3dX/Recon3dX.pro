#-------------------------------------------------
#
# Project created by QtCreator 2015-03-19T15:06:14
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Recon3dX
TEMPLATE = app

DESTDIR = "../bin"

include(opencv.pri)
include(pcl.pri)

SOURCES += main.cpp \
    mainwindow.cpp \
    processor.cpp \
    VideoInput.cpp \
    ImageLabel.cpp \
    io_util.cpp \
    CameraObj.cpp \
    ProjectorWidget.cpp \
    mymatcher.cpp

HEADERS  += mainwindow.h \
    processor.h \
    VideoInput.hpp \
    ImageLabel.hpp \
    io_util.hpp \
    CameraObj.hpp \
    ProjectorWidget.hpp \
    mymatcher.h

FORMS    += mainwindow.ui

RESOURCES += \
    res.qrc
