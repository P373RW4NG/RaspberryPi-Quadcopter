#-------------------------------------------------
#
# Project created by QtCreator 2017-01-12T22:08:39
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = client-GUI
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/local/Cellar/boost/1.64.0_1/include

LIBS+= -L/usr/local/Cellar/boost/1.64.0_1/lib  -lboost_system\
                                               -lboost_date_time\
                                               -lboost_thread-mt


LIBS+= -L/usr/local/lib -lopencv_world

