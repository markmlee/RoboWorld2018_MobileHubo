QT += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

CONFIG(debug, debug|release) {
    DESTDIR = ../../exe
} else {
    DESTDIR = ../../exe
}


TARGET = PODOLauncher
CONFIG -= console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    LauncherDialog.cpp

FORMS += \
    LauncherDialog.ui

HEADERS += \
    LauncherDialog.h

RESOURCES += \
    icon.qrc
