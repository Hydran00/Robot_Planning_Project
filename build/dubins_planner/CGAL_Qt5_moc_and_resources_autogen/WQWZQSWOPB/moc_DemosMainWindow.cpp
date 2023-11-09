/****************************************************************************
** Meta object code from reading C++ file 'DemosMainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../../../../usr/include/CGAL/Qt/DemosMainWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'DemosMainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CGAL__Qt__DemosMainWindow_t {
    QByteArrayData data[15];
    char stringdata0[226];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CGAL__Qt__DemosMainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CGAL__Qt__DemosMainWindow_t qt_meta_stringdata_CGAL__Qt__DemosMainWindow = {
    {
QT_MOC_LITERAL(0, 0, 25), // "CGAL::Qt::DemosMainWindow"
QT_MOC_LITERAL(1, 26, 14), // "openRecentFile"
QT_MOC_LITERAL(2, 41, 0), // ""
QT_MOC_LITERAL(3, 42, 8), // "filename"
QT_MOC_LITERAL(4, 51, 25), // "setMaxNumberOfRecentFiles"
QT_MOC_LITERAL(5, 77, 18), // "setUseAntialiasing"
QT_MOC_LITERAL(6, 96, 7), // "checked"
QT_MOC_LITERAL(7, 104, 12), // "setUseOpenGL"
QT_MOC_LITERAL(8, 117, 14), // "popupAboutCGAL"
QT_MOC_LITERAL(9, 132, 14), // "popupAboutDemo"
QT_MOC_LITERAL(10, 147, 9), // "exportSVG"
QT_MOC_LITERAL(11, 157, 18), // "openRecentFile_aux"
QT_MOC_LITERAL(12, 176, 16), // "addToRecentFiles"
QT_MOC_LITERAL(13, 193, 8), // "fileName"
QT_MOC_LITERAL(14, 202, 23) // "updateRecentFileActions"

    },
    "CGAL::Qt::DemosMainWindow\0openRecentFile\0"
    "\0filename\0setMaxNumberOfRecentFiles\0"
    "setUseAntialiasing\0checked\0setUseOpenGL\0"
    "popupAboutCGAL\0popupAboutDemo\0exportSVG\0"
    "openRecentFile_aux\0addToRecentFiles\0"
    "fileName\0updateRecentFileActions"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CGAL__Qt__DemosMainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   64,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    1,   67,    2, 0x0a /* Public */,
       5,    1,   70,    2, 0x09 /* Protected */,
       7,    1,   73,    2, 0x09 /* Protected */,
       8,    0,   76,    2, 0x09 /* Protected */,
       9,    0,   77,    2, 0x09 /* Protected */,
      10,    0,   78,    2, 0x09 /* Protected */,
      11,    0,   79,    2, 0x09 /* Protected */,
      12,    1,   80,    2, 0x09 /* Protected */,
      14,    0,   83,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,

 // slots: parameters
    QMetaType::Void, QMetaType::UInt,    2,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void,

       0        // eod
};

void CGAL::Qt::DemosMainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DemosMainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->openRecentFile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->setMaxNumberOfRecentFiles((*reinterpret_cast< const uint(*)>(_a[1]))); break;
        case 2: _t->setUseAntialiasing((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->setUseOpenGL((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->popupAboutCGAL(); break;
        case 5: _t->popupAboutDemo(); break;
        case 6: _t->exportSVG(); break;
        case 7: _t->openRecentFile_aux(); break;
        case 8: _t->addToRecentFiles((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 9: _t->updateRecentFileActions(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (DemosMainWindow::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DemosMainWindow::openRecentFile)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CGAL::Qt::DemosMainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_CGAL__Qt__DemosMainWindow.data,
    qt_meta_data_CGAL__Qt__DemosMainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CGAL::Qt::DemosMainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CGAL::Qt::DemosMainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CGAL__Qt__DemosMainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int CGAL::Qt::DemosMainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void CGAL::Qt::DemosMainWindow::openRecentFile(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
