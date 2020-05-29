#-------------------------------------------------
#
# Project created by QtCreator 2016-06-29T17:10:32
#
#-------------------------------------------------

QT       += core gui sql

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = DeviceManager
TEMPLATE = app


CONFIG(debug, debug|release) {
    DESTDIR = ../../exe
} else {
    DESTDIR = ../../exe
}


QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
QMAKE_CXXFLAGS += -I/usr/include/xenomai
QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt


INCLUDEPATH += \
    /usr/include/xenomai/cobalt \
    /usr/include/xenomai \
    /usr/include/xenomai/alchemy


LIBS    += \
    -lalchemy \
    -lcopperplate \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan



SOURCES += main.cpp\
        MainWindow.cpp \
    Panel_MC.cpp \
    RBCAN.cpp \
    RBDataBase.cpp \
    RBMotorController.cpp \
    Panel_FT.cpp \
    RBFTSensor.cpp \
    qcustomplot.cpp \
    RBGraph.cpp

HEADERS  += MainWindow.h \
    Panel_MC.h \
    RBCAN.h \
    RBLog.h \
    RBDataBase.h \
    RBDataType.h \
    RBMotorController.h \
    Panel_FT.h \
    RBFTSensor.h \
    qcustomplot.h \
    RBGraph.h

FORMS    += MainWindow.ui \
    Panel_MC.ui \
    Panel_FT.ui \
    RBGraph.ui
