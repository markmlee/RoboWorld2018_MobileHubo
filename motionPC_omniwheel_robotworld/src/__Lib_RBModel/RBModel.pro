#-------------------------------------------------
#
# Project created by QtCreator 2016-01-04T18:52:19
#
#-------------------------------------------------

QT      -= core gui
QT	+= core opengl

TARGET = RBModel
TEMPLATE = lib
CONFIG += staticlib

SOURCES += *.cpp

HEADERS += *.h \
        isnl/base/*.h \
        isnl/math/*.h \
        isnl/util/*.h \
        isnl/opengl/*.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
