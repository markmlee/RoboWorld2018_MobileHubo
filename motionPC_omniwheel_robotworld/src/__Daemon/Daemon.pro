QT  += sql

TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle

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


SOURCES += main.cpp \
    RBDataBase.cpp \
    RBMotorController.cpp \
    RBCAN.cpp \
    RBProcessManager.cpp \
    RBFTSensor.cpp \
    RBIMUSensor.cpp \
    RBSmartPower.cpp \
    RBOpticFlowSensor.cpp \
    RBFOGSensor.cpp \
    RBRawLAN.cpp

HEADERS += \
    RBDataBase.h \
    RBLog.h \
    RBDataType.h \
    RBMotorController.h \
    RBCAN.h \
    RBProcessManager.h \
    RBFTSensor.h \
    RBIMUSensor.h \
    RBSmartPower.h \
    RBOpticFlowSensor.h \
    RBFOGSensor.h \
    RBRawLAN.h \
    JointInformation.h \
    ExternHeader.h


