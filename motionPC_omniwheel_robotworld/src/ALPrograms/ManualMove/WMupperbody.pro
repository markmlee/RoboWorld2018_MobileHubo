#-------------------------------------------------
#
# Project created by QtCreator 2014-02-11T10:41:29
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = ManualMove
CONFIG   += console
CONFIG   -= app_bundle

CONFIG(debug, debug|release) {
    DESTDIR = ../PODO_PROC_Build
} else {
    DESTDIR = ../PODO_PROC_Build
}

TEMPLATE = app


INCLUDEPATH += /usr/include/xenomai \

LIBS        += -lrtdm \
               -lnative \
               -lxenomai \
               -lpthread_rt \
               -lpcan \
               -lrt \
               -L../../SHARE -lKINE_DRC_HUBO2 \
                -L../../SHARE -lik_math2\

SOURCES += main.cpp \
    joint.cpp \
    taskmotion.cpp \
    BasicTrajectory.cpp \
    MotionChecker.cpp

HEADERS += \
    BaseJoint.h \
    joint.h \
    taskmotion.h \
    taskGeneral.h \
    BasicTrajectory.h \
    BasicMatrix.h \
    BasicMath.h \
    ManualMove.h \
    MotionChecker.h

