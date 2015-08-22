#-------------------------------------------------
#
# Project created by QtCreator 2014-11-25T13:00:35
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RobotUI
TEMPLATE = app

QMAKE_CXXFLAGS += -O2\
          -std=c++11

INCLUDEPATH += /home/sun/Projects/Aris/src\
               /home/sun/Projects/Aris/src/Aris_Control\
               /home/sun/Projects/Aris/src/Aris_Core\
               /opt/etherlab/include\
               /usr/xenomai/include

LIBS += /home/sun/Projects/Aris/build/lib/libAris_Core.a\
        /home/sun/Projects/Aris/build/lib/libAris_Control.a

SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h \
    ConnectionSetting.h

FORMS    += mainwindow.ui
