/****************************************************************************
** Meta object code from reading C++ file 'manipulatedCameraFrame.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../../../../usr/include/CGAL/Qt/manipulatedCameraFrame.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'manipulatedCameraFrame.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CGAL__qglviewer__ManipulatedCameraFrame_t {
    QByteArrayData data[9];
    char stringdata0[98];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CGAL__qglviewer__ManipulatedCameraFrame_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CGAL__qglviewer__ManipulatedCameraFrame_t qt_meta_stringdata_CGAL__qglviewer__ManipulatedCameraFrame = {
    {
QT_MOC_LITERAL(0, 0, 39), // "CGAL::qglviewer::ManipulatedC..."
QT_MOC_LITERAL(1, 40, 11), // "setFlySpeed"
QT_MOC_LITERAL(2, 52, 0), // ""
QT_MOC_LITERAL(3, 53, 5), // "speed"
QT_MOC_LITERAL(4, 59, 16), // "setSceneUpVector"
QT_MOC_LITERAL(5, 76, 3), // "Vec"
QT_MOC_LITERAL(6, 80, 2), // "up"
QT_MOC_LITERAL(7, 83, 4), // "spin"
QT_MOC_LITERAL(8, 88, 9) // "flyUpdate"

    },
    "CGAL::qglviewer::ManipulatedCameraFrame\0"
    "setFlySpeed\0\0speed\0setSceneUpVector\0"
    "Vec\0up\0spin\0flyUpdate"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CGAL__qglviewer__ManipulatedCameraFrame[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x0a /* Public */,
       4,    1,   37,    2, 0x0a /* Public */,
       7,    0,   40,    2, 0x09 /* Protected */,
       8,    0,   41,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::QReal,    3,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void CGAL::qglviewer::ManipulatedCameraFrame::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ManipulatedCameraFrame *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->setFlySpeed((*reinterpret_cast< qreal(*)>(_a[1]))); break;
        case 1: _t->setSceneUpVector((*reinterpret_cast< const Vec(*)>(_a[1]))); break;
        case 2: _t->spin(); break;
        case 3: _t->flyUpdate(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CGAL::qglviewer::ManipulatedCameraFrame::staticMetaObject = { {
    QMetaObject::SuperData::link<ManipulatedFrame::staticMetaObject>(),
    qt_meta_stringdata_CGAL__qglviewer__ManipulatedCameraFrame.data,
    qt_meta_data_CGAL__qglviewer__ManipulatedCameraFrame,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CGAL::qglviewer::ManipulatedCameraFrame::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CGAL::qglviewer::ManipulatedCameraFrame::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CGAL__qglviewer__ManipulatedCameraFrame.stringdata0))
        return static_cast<void*>(this);
    return ManipulatedFrame::qt_metacast(_clname);
}

int CGAL::qglviewer::ManipulatedCameraFrame::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ManipulatedFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
