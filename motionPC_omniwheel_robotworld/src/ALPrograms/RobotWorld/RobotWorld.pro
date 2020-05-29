#-------------------------------------------------
#
# Project created by QtCreator 2014-02-11T10:41:29
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = RobotWorld
CONFIG   += console
CONFIG   -= app_bundle

CONFIG(debug, debug|release) {
    DESTDIR = ../PODO_PROC_Build
} else {
    DESTDIR = ../PODO_PROC_Build
}

TEMPLATE = app

QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
QMAKE_CXXFLAGS += -I/usr/include/xenomai
QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt



INCLUDEPATH += \
    /usr/include/xenomai/cobalt \
    /usr/include/xenomai \
    /usr/include/xenomai/alchemy


LIBS        += \
    -lalchemy \
    -lcopperplate \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan \
   -L../../../share/Libs       -lKINE_DRC_HUBO2 \
   -L../../../share/Libs	-lik_math2 \

SOURCES += main.cpp \
    BasicFiles/joint.cpp \
    BasicFiles/BasicTrajectory.cpp \
    BasicFiles/taskmotion.cpp \
    BasicFiles/ManualCAN.cpp \
    Drinkstock.cpp

HEADERS += \
    BasicFiles/joint.h \
    BasicFiles/BasicMath.h \
    BasicFiles/BasicMatrix.h \
    BasicFiles/BasicTrajectory.h \
    BasicFiles/taskGeneral.h \
    BasicFiles/taskmotion.h \
    BasicFiles/RBLog.h \
    BasicFiles/ManualCAN.h \
    MainHeader.h \
    Drinkstock.h

