#-------------------------------------------------
#
# Project created by QtCreator 2014-02-11T10:41:29
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = WalkReady
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
   #-L../../SHARE/Libs	-lik_math2 \
   -L../../../share/Libs       -lKINE_DRC_HUBO4 \
   -L../../../share/Libs	-lik_math4 \
#                -L../../SHARE/Libs	-lKINE_DRC_HUBO_TJ \
#               -L../../SHARE/Libs	-lik_math2 \


SOURCES += main.cpp \
    joint.cpp \
    ManualCAN.cpp

HEADERS += \
    joint.h \
    RBLog.h \
    kine_drc_hubo_tj.h \
    ManualCAN.h

