/****************************************************************************
** Meta object code from reading C++ file 'LauncherDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/__PODOLauncher/LauncherDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'LauncherDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_LauncherDialog_t {
    QByteArrayData data[11];
    char stringdata0[217];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_LauncherDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_LauncherDialog_t qt_meta_stringdata_LauncherDialog = {
    {
QT_MOC_LITERAL(0, 0, 14), // "LauncherDialog"
QT_MOC_LITERAL(1, 15, 28), // "on_BTN_CHANGE_DAEMON_clicked"
QT_MOC_LITERAL(2, 44, 0), // ""
QT_MOC_LITERAL(3, 45, 25), // "on_BTN_CHANGE_GUI_clicked"
QT_MOC_LITERAL(4, 71, 19), // "on_RB_ROBOT_toggled"
QT_MOC_LITERAL(5, 91, 7), // "checked"
QT_MOC_LITERAL(6, 99, 27), // "on_BTN_START_DAEMON_clicked"
QT_MOC_LITERAL(7, 127, 26), // "on_BTN_STOP_DAEMON_clicked"
QT_MOC_LITERAL(8, 154, 24), // "on_BTN_START_GUI_clicked"
QT_MOC_LITERAL(9, 179, 23), // "on_BTN_STOP_GUI_clicked"
QT_MOC_LITERAL(10, 203, 13) // "on_showNormal"

    },
    "LauncherDialog\0on_BTN_CHANGE_DAEMON_clicked\0"
    "\0on_BTN_CHANGE_GUI_clicked\0"
    "on_RB_ROBOT_toggled\0checked\0"
    "on_BTN_START_DAEMON_clicked\0"
    "on_BTN_STOP_DAEMON_clicked\0"
    "on_BTN_START_GUI_clicked\0"
    "on_BTN_STOP_GUI_clicked\0on_showNormal"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_LauncherDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x08 /* Private */,
       3,    0,   55,    2, 0x08 /* Private */,
       4,    1,   56,    2, 0x08 /* Private */,
       6,    0,   59,    2, 0x08 /* Private */,
       7,    0,   60,    2, 0x08 /* Private */,
       8,    0,   61,    2, 0x08 /* Private */,
       9,    0,   62,    2, 0x08 /* Private */,
      10,    0,   63,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void LauncherDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        LauncherDialog *_t = static_cast<LauncherDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_BTN_CHANGE_DAEMON_clicked(); break;
        case 1: _t->on_BTN_CHANGE_GUI_clicked(); break;
        case 2: _t->on_RB_ROBOT_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_BTN_START_DAEMON_clicked(); break;
        case 4: _t->on_BTN_STOP_DAEMON_clicked(); break;
        case 5: _t->on_BTN_START_GUI_clicked(); break;
        case 6: _t->on_BTN_STOP_GUI_clicked(); break;
        case 7: _t->on_showNormal(); break;
        default: ;
        }
    }
}

const QMetaObject LauncherDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_LauncherDialog.data,
      qt_meta_data_LauncherDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *LauncherDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *LauncherDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_LauncherDialog.stringdata0))
        return static_cast<void*>(const_cast< LauncherDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int LauncherDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
