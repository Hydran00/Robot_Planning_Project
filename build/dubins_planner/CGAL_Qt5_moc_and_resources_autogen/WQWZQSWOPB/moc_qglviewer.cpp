/****************************************************************************
** Meta object code from reading C++ file 'qglviewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../../../../usr/include/CGAL/Qt/qglviewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qglviewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CGAL__QGLViewer_t {
    QByteArrayData data[119];
    char stringdata0[1691];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CGAL__QGLViewer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CGAL__QGLViewer_t qt_meta_stringdata_CGAL__QGLViewer = {
    {
QT_MOC_LITERAL(0, 0, 15), // "CGAL::QGLViewer"
QT_MOC_LITERAL(1, 16, 17), // "viewerInitialized"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 10), // "drawNeeded"
QT_MOC_LITERAL(4, 46, 12), // "drawFinished"
QT_MOC_LITERAL(5, 59, 9), // "automatic"
QT_MOC_LITERAL(6, 69, 13), // "animateNeeded"
QT_MOC_LITERAL(7, 83, 12), // "helpRequired"
QT_MOC_LITERAL(8, 96, 18), // "axisIsDrawnChanged"
QT_MOC_LITERAL(9, 115, 5), // "drawn"
QT_MOC_LITERAL(10, 121, 18), // "gridIsDrawnChanged"
QT_MOC_LITERAL(11, 140, 21), // "FPSIsDisplayedChanged"
QT_MOC_LITERAL(12, 162, 9), // "displayed"
QT_MOC_LITERAL(13, 172, 20), // "textIsEnabledChanged"
QT_MOC_LITERAL(14, 193, 7), // "enabled"
QT_MOC_LITERAL(15, 201, 21), // "cameraIsEditedChanged"
QT_MOC_LITERAL(16, 223, 6), // "edited"
QT_MOC_LITERAL(17, 230, 13), // "pointSelected"
QT_MOC_LITERAL(18, 244, 18), // "const QMouseEvent*"
QT_MOC_LITERAL(19, 263, 1), // "e"
QT_MOC_LITERAL(20, 265, 19), // "mouseGrabberChanged"
QT_MOC_LITERAL(21, 285, 24), // "qglviewer::MouseGrabber*"
QT_MOC_LITERAL(22, 310, 12), // "mouseGrabber"
QT_MOC_LITERAL(23, 323, 18), // "contextIsDestroyed"
QT_MOC_LITERAL(24, 342, 14), // "needNewContext"
QT_MOC_LITERAL(25, 357, 14), // "setAxisIsDrawn"
QT_MOC_LITERAL(26, 372, 4), // "draw"
QT_MOC_LITERAL(27, 377, 14), // "setGridIsDrawn"
QT_MOC_LITERAL(28, 392, 17), // "setFPSIsDisplayed"
QT_MOC_LITERAL(29, 410, 7), // "display"
QT_MOC_LITERAL(30, 418, 16), // "setTextIsEnabled"
QT_MOC_LITERAL(31, 435, 6), // "enable"
QT_MOC_LITERAL(32, 442, 17), // "setCameraIsEdited"
QT_MOC_LITERAL(33, 460, 4), // "edit"
QT_MOC_LITERAL(34, 465, 17), // "toggleAxisIsDrawn"
QT_MOC_LITERAL(35, 483, 17), // "toggleGridIsDrawn"
QT_MOC_LITERAL(36, 501, 20), // "toggleFPSIsDisplayed"
QT_MOC_LITERAL(37, 522, 19), // "toggleTextIsEnabled"
QT_MOC_LITERAL(38, 542, 20), // "toggleCameraIsEdited"
QT_MOC_LITERAL(39, 563, 18), // "setBackgroundColor"
QT_MOC_LITERAL(40, 582, 5), // "color"
QT_MOC_LITERAL(41, 588, 18), // "setForegroundColor"
QT_MOC_LITERAL(42, 607, 14), // "setSceneRadius"
QT_MOC_LITERAL(43, 622, 6), // "radius"
QT_MOC_LITERAL(44, 629, 14), // "setSceneCenter"
QT_MOC_LITERAL(45, 644, 14), // "qglviewer::Vec"
QT_MOC_LITERAL(46, 659, 6), // "center"
QT_MOC_LITERAL(47, 666, 19), // "setSceneBoundingBox"
QT_MOC_LITERAL(48, 686, 3), // "min"
QT_MOC_LITERAL(49, 690, 3), // "max"
QT_MOC_LITERAL(50, 694, 15), // "showEntireScene"
QT_MOC_LITERAL(51, 710, 9), // "setCamera"
QT_MOC_LITERAL(52, 720, 23), // "qglviewer::Camera*const"
QT_MOC_LITERAL(53, 744, 6), // "camera"
QT_MOC_LITERAL(54, 751, 19), // "setManipulatedFrame"
QT_MOC_LITERAL(55, 771, 28), // "qglviewer::ManipulatedFrame*"
QT_MOC_LITERAL(56, 800, 5), // "frame"
QT_MOC_LITERAL(57, 806, 15), // "setMouseGrabber"
QT_MOC_LITERAL(58, 822, 13), // "setFullScreen"
QT_MOC_LITERAL(59, 836, 10), // "fullScreen"
QT_MOC_LITERAL(60, 847, 16), // "toggleFullScreen"
QT_MOC_LITERAL(61, 864, 16), // "toggleCameraMode"
QT_MOC_LITERAL(62, 881, 19), // "copyBufferToTexture"
QT_MOC_LITERAL(63, 901, 5), // "GLint"
QT_MOC_LITERAL(64, 907, 6), // "GLenum"
QT_MOC_LITERAL(65, 914, 18), // "setAnimationPeriod"
QT_MOC_LITERAL(66, 933, 6), // "period"
QT_MOC_LITERAL(67, 940, 14), // "startAnimation"
QT_MOC_LITERAL(68, 955, 13), // "stopAnimation"
QT_MOC_LITERAL(69, 969, 7), // "animate"
QT_MOC_LITERAL(70, 977, 15), // "toggleAnimation"
QT_MOC_LITERAL(71, 993, 4), // "help"
QT_MOC_LITERAL(72, 998, 14), // "aboutQGLViewer"
QT_MOC_LITERAL(73, 1013, 6), // "select"
QT_MOC_LITERAL(74, 1020, 5), // "event"
QT_MOC_LITERAL(75, 1026, 5), // "point"
QT_MOC_LITERAL(76, 1032, 19), // "setSelectBufferSize"
QT_MOC_LITERAL(77, 1052, 4), // "size"
QT_MOC_LITERAL(78, 1057, 20), // "setSelectRegionWidth"
QT_MOC_LITERAL(79, 1078, 5), // "width"
QT_MOC_LITERAL(80, 1084, 21), // "setSelectRegionHeight"
QT_MOC_LITERAL(81, 1106, 6), // "height"
QT_MOC_LITERAL(82, 1113, 15), // "setSelectedName"
QT_MOC_LITERAL(83, 1129, 2), // "id"
QT_MOC_LITERAL(84, 1132, 11), // "setShortcut"
QT_MOC_LITERAL(85, 1144, 25), // "qglviewer::KeyboardAction"
QT_MOC_LITERAL(86, 1170, 6), // "action"
QT_MOC_LITERAL(87, 1177, 3), // "key"
QT_MOC_LITERAL(88, 1181, 14), // "::Qt::Modifier"
QT_MOC_LITERAL(89, 1196, 8), // "modifier"
QT_MOC_LITERAL(90, 1205, 9), // "::Qt::Key"
QT_MOC_LITERAL(91, 1215, 17), // "setKeyDescription"
QT_MOC_LITERAL(92, 1233, 11), // "description"
QT_MOC_LITERAL(93, 1245, 22), // "::Qt::KeyboardModifier"
QT_MOC_LITERAL(94, 1268, 14), // "clearShortcuts"
QT_MOC_LITERAL(95, 1283, 10), // "setPathKey"
QT_MOC_LITERAL(96, 1294, 5), // "index"
QT_MOC_LITERAL(97, 1300, 28), // "setPlayPathKeyboardModifiers"
QT_MOC_LITERAL(98, 1329, 23), // "::Qt::KeyboardModifiers"
QT_MOC_LITERAL(99, 1353, 9), // "modifiers"
QT_MOC_LITERAL(100, 1363, 31), // "setAddKeyFrameKeyboardModifiers"
QT_MOC_LITERAL(101, 1395, 15), // "setMouseBinding"
QT_MOC_LITERAL(102, 1411, 17), // "::Qt::MouseButton"
QT_MOC_LITERAL(103, 1429, 7), // "buttons"
QT_MOC_LITERAL(104, 1437, 23), // "qglviewer::MouseHandler"
QT_MOC_LITERAL(105, 1461, 7), // "handler"
QT_MOC_LITERAL(106, 1469, 22), // "qglviewer::MouseAction"
QT_MOC_LITERAL(107, 1492, 14), // "withConstraint"
QT_MOC_LITERAL(108, 1507, 6), // "button"
QT_MOC_LITERAL(109, 1514, 22), // "qglviewer::ClickAction"
QT_MOC_LITERAL(110, 1537, 11), // "doubleClick"
QT_MOC_LITERAL(111, 1549, 18), // "::Qt::MouseButtons"
QT_MOC_LITERAL(112, 1568, 13), // "buttonsBefore"
QT_MOC_LITERAL(113, 1582, 15), // "setWheelBinding"
QT_MOC_LITERAL(114, 1598, 26), // "setMouseBindingDescription"
QT_MOC_LITERAL(115, 1625, 18), // "clearMouseBindings"
QT_MOC_LITERAL(116, 1644, 16), // "resetVisualHints"
QT_MOC_LITERAL(117, 1661, 17), // "delayedFullScreen"
QT_MOC_LITERAL(118, 1679, 11) // "hideMessage"

    },
    "CGAL::QGLViewer\0viewerInitialized\0\0"
    "drawNeeded\0drawFinished\0automatic\0"
    "animateNeeded\0helpRequired\0"
    "axisIsDrawnChanged\0drawn\0gridIsDrawnChanged\0"
    "FPSIsDisplayedChanged\0displayed\0"
    "textIsEnabledChanged\0enabled\0"
    "cameraIsEditedChanged\0edited\0pointSelected\0"
    "const QMouseEvent*\0e\0mouseGrabberChanged\0"
    "qglviewer::MouseGrabber*\0mouseGrabber\0"
    "contextIsDestroyed\0needNewContext\0"
    "setAxisIsDrawn\0draw\0setGridIsDrawn\0"
    "setFPSIsDisplayed\0display\0setTextIsEnabled\0"
    "enable\0setCameraIsEdited\0edit\0"
    "toggleAxisIsDrawn\0toggleGridIsDrawn\0"
    "toggleFPSIsDisplayed\0toggleTextIsEnabled\0"
    "toggleCameraIsEdited\0setBackgroundColor\0"
    "color\0setForegroundColor\0setSceneRadius\0"
    "radius\0setSceneCenter\0qglviewer::Vec\0"
    "center\0setSceneBoundingBox\0min\0max\0"
    "showEntireScene\0setCamera\0"
    "qglviewer::Camera*const\0camera\0"
    "setManipulatedFrame\0qglviewer::ManipulatedFrame*\0"
    "frame\0setMouseGrabber\0setFullScreen\0"
    "fullScreen\0toggleFullScreen\0"
    "toggleCameraMode\0copyBufferToTexture\0"
    "GLint\0GLenum\0setAnimationPeriod\0period\0"
    "startAnimation\0stopAnimation\0animate\0"
    "toggleAnimation\0help\0aboutQGLViewer\0"
    "select\0event\0point\0setSelectBufferSize\0"
    "size\0setSelectRegionWidth\0width\0"
    "setSelectRegionHeight\0height\0"
    "setSelectedName\0id\0setShortcut\0"
    "qglviewer::KeyboardAction\0action\0key\0"
    "::Qt::Modifier\0modifier\0::Qt::Key\0"
    "setKeyDescription\0description\0"
    "::Qt::KeyboardModifier\0clearShortcuts\0"
    "setPathKey\0index\0setPlayPathKeyboardModifiers\0"
    "::Qt::KeyboardModifiers\0modifiers\0"
    "setAddKeyFrameKeyboardModifiers\0"
    "setMouseBinding\0::Qt::MouseButton\0"
    "buttons\0qglviewer::MouseHandler\0handler\0"
    "qglviewer::MouseAction\0withConstraint\0"
    "button\0qglviewer::ClickAction\0doubleClick\0"
    "::Qt::MouseButtons\0buttonsBefore\0"
    "setWheelBinding\0setMouseBindingDescription\0"
    "clearMouseBindings\0resetVisualHints\0"
    "delayedFullScreen\0hideMessage"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CGAL__QGLViewer[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      91,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      14,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,  469,    2, 0x06 /* Public */,
       3,    0,  470,    2, 0x06 /* Public */,
       4,    1,  471,    2, 0x06 /* Public */,
       6,    0,  474,    2, 0x06 /* Public */,
       7,    0,  475,    2, 0x06 /* Public */,
       8,    1,  476,    2, 0x06 /* Public */,
      10,    1,  479,    2, 0x06 /* Public */,
      11,    1,  482,    2, 0x06 /* Public */,
      13,    1,  485,    2, 0x06 /* Public */,
      15,    1,  488,    2, 0x06 /* Public */,
      17,    1,  491,    2, 0x06 /* Public */,
      20,    1,  494,    2, 0x06 /* Public */,
      23,    0,  497,    2, 0x06 /* Public */,
      24,    0,  498,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      25,    1,  499,    2, 0x0a /* Public */,
      25,    0,  502,    2, 0x2a /* Public | MethodCloned */,
      27,    1,  503,    2, 0x0a /* Public */,
      27,    0,  506,    2, 0x2a /* Public | MethodCloned */,
      28,    1,  507,    2, 0x0a /* Public */,
      28,    0,  510,    2, 0x2a /* Public | MethodCloned */,
      30,    1,  511,    2, 0x0a /* Public */,
      30,    0,  514,    2, 0x2a /* Public | MethodCloned */,
      32,    1,  515,    2, 0x0a /* Public */,
      32,    0,  518,    2, 0x2a /* Public | MethodCloned */,
      34,    0,  519,    2, 0x0a /* Public */,
      35,    0,  520,    2, 0x0a /* Public */,
      36,    0,  521,    2, 0x0a /* Public */,
      37,    0,  522,    2, 0x0a /* Public */,
      38,    0,  523,    2, 0x0a /* Public */,
      39,    1,  524,    2, 0x0a /* Public */,
      41,    1,  527,    2, 0x0a /* Public */,
      42,    1,  530,    2, 0x0a /* Public */,
      44,    1,  533,    2, 0x0a /* Public */,
      47,    2,  536,    2, 0x0a /* Public */,
      50,    0,  541,    2, 0x0a /* Public */,
      51,    1,  542,    2, 0x0a /* Public */,
      54,    1,  545,    2, 0x0a /* Public */,
      57,    1,  548,    2, 0x0a /* Public */,
      58,    1,  551,    2, 0x0a /* Public */,
      58,    0,  554,    2, 0x2a /* Public | MethodCloned */,
      60,    0,  555,    2, 0x0a /* Public */,
      61,    0,  556,    2, 0x0a /* Public */,
      62,    2,  557,    2, 0x0a /* Public */,
      62,    1,  562,    2, 0x2a /* Public | MethodCloned */,
      65,    1,  565,    2, 0x0a /* Public */,
      67,    0,  568,    2, 0x0a /* Public */,
      68,    0,  569,    2, 0x0a /* Public */,
      69,    0,  570,    2, 0x0a /* Public */,
      70,    0,  571,    2, 0x0a /* Public */,
      71,    0,  572,    2, 0x0a /* Public */,
      72,    0,  573,    2, 0x0a /* Public */,
      73,    1,  574,    2, 0x0a /* Public */,
      73,    1,  577,    2, 0x0a /* Public */,
      76,    1,  580,    2, 0x0a /* Public */,
      78,    1,  583,    2, 0x0a /* Public */,
      80,    1,  586,    2, 0x0a /* Public */,
      82,    1,  589,    2, 0x0a /* Public */,
      84,    2,  592,    2, 0x0a /* Public */,
      84,    3,  597,    2, 0x0a /* Public */,
      91,    2,  604,    2, 0x0a /* Public */,
      91,    3,  609,    2, 0x0a /* Public */,
      91,    3,  616,    2, 0x0a /* Public */,
      94,    0,  623,    2, 0x0a /* Public */,
      95,    2,  624,    2, 0x0a /* Public */,
      95,    1,  629,    2, 0x2a /* Public | MethodCloned */,
      97,    1,  632,    2, 0x0a /* Public */,
     100,    1,  635,    2, 0x0a /* Public */,
     101,    5,  638,    2, 0x0a /* Public */,
     101,    4,  649,    2, 0x2a /* Public | MethodCloned */,
     101,    5,  658,    2, 0x0a /* Public */,
     101,    4,  669,    2, 0x2a /* Public | MethodCloned */,
     101,    3,  678,    2, 0x2a /* Public | MethodCloned */,
     113,    4,  685,    2, 0x0a /* Public */,
     113,    3,  694,    2, 0x2a /* Public | MethodCloned */,
     114,    5,  701,    2, 0x0a /* Public */,
     114,    4,  712,    2, 0x2a /* Public | MethodCloned */,
     114,    3,  721,    2, 0x2a /* Public | MethodCloned */,
     101,    6,  728,    2, 0x0a /* Public */,
     101,    5,  741,    2, 0x2a /* Public | MethodCloned */,
     101,    6,  752,    2, 0x0a /* Public */,
     101,    5,  765,    2, 0x2a /* Public | MethodCloned */,
     101,    4,  776,    2, 0x2a /* Public | MethodCloned */,
     113,    5,  785,    2, 0x0a /* Public */,
     113,    4,  796,    2, 0x2a /* Public | MethodCloned */,
     114,    6,  805,    2, 0x0a /* Public */,
     114,    5,  818,    2, 0x2a /* Public | MethodCloned */,
     114,    4,  829,    2, 0x2a /* Public | MethodCloned */,
     115,    0,  838,    2, 0x0a /* Public */,
     116,    0,  839,    2, 0x0a /* Public */,
     117,    0,  840,    2, 0x08 /* Private */,
     118,    0,  841,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    9,
    QMetaType::Void, QMetaType::Bool,    9,
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   16,
    QMetaType::Void, 0x80000000 | 18,   19,
    QMetaType::Void, 0x80000000 | 21,   22,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,   26,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   26,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   29,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   31,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   33,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QColor,   40,
    QMetaType::Void, QMetaType::QColor,   40,
    QMetaType::Void, QMetaType::QReal,   43,
    QMetaType::Void, 0x80000000 | 45,   46,
    QMetaType::Void, 0x80000000 | 45, 0x80000000 | 45,   48,   49,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 52,   53,
    QMetaType::Void, 0x80000000 | 55,   56,
    QMetaType::Void, 0x80000000 | 21,   22,
    QMetaType::Void, QMetaType::Bool,   59,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 63, 0x80000000 | 64,    2,    2,
    QMetaType::Void, 0x80000000 | 63,    2,
    QMetaType::Void, QMetaType::Int,   66,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 18,   74,
    QMetaType::Void, QMetaType::QPoint,   75,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   79,
    QMetaType::Void, QMetaType::Int,   81,
    QMetaType::Void, QMetaType::Int,   83,
    QMetaType::Void, 0x80000000 | 85, QMetaType::UInt,   86,   87,
    QMetaType::Void, 0x80000000 | 85, 0x80000000 | 88, 0x80000000 | 90,   86,   89,   87,
    QMetaType::Void, QMetaType::UInt, QMetaType::QString,   87,   92,
    QMetaType::Void, 0x80000000 | 93, 0x80000000 | 90, QMetaType::QString,   89,   87,   92,
    QMetaType::Void, 0x80000000 | 88, 0x80000000 | 90, QMetaType::QString,   89,   87,   92,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::UInt,   87,   96,
    QMetaType::Void, QMetaType::Int,   87,
    QMetaType::Void, 0x80000000 | 98,   99,
    QMetaType::Void, 0x80000000 | 98,   99,
    QMetaType::Void, 0x80000000 | 98, 0x80000000 | 102, 0x80000000 | 104, 0x80000000 | 106, QMetaType::Bool,   99,  103,  105,   86,  107,
    QMetaType::Void, 0x80000000 | 98, 0x80000000 | 102, 0x80000000 | 104, 0x80000000 | 106,   99,  103,  105,   86,
    QMetaType::Void, 0x80000000 | 98, 0x80000000 | 102, 0x80000000 | 109, QMetaType::Bool, 0x80000000 | 111,   99,  108,   86,  110,  112,
    QMetaType::Void, 0x80000000 | 98, 0x80000000 | 102, 0x80000000 | 109, QMetaType::Bool,   99,  108,   86,  110,
    QMetaType::Void, 0x80000000 | 98, 0x80000000 | 102, 0x80000000 | 109,   99,  108,   86,
    QMetaType::Void, 0x80000000 | 98, 0x80000000 | 104, 0x80000000 | 106, QMetaType::Bool,   99,  105,   86,  107,
    QMetaType::Void, 0x80000000 | 98, 0x80000000 | 104, 0x80000000 | 106,   99,  105,   86,
    QMetaType::Void, 0x80000000 | 98, 0x80000000 | 102, QMetaType::QString, QMetaType::Bool, 0x80000000 | 111,   99,  108,   92,  110,  112,
    QMetaType::Void, 0x80000000 | 98, 0x80000000 | 102, QMetaType::QString, QMetaType::Bool,   99,  108,   92,  110,
    QMetaType::Void, 0x80000000 | 98, 0x80000000 | 102, QMetaType::QString,   99,  108,   92,
    QMetaType::Void, 0x80000000 | 90, 0x80000000 | 98, 0x80000000 | 102, 0x80000000 | 104, 0x80000000 | 106, QMetaType::Bool,   87,   99,  103,  105,   86,  107,
    QMetaType::Void, 0x80000000 | 90, 0x80000000 | 98, 0x80000000 | 102, 0x80000000 | 104, 0x80000000 | 106,   87,   99,  103,  105,   86,
    QMetaType::Void, 0x80000000 | 90, 0x80000000 | 98, 0x80000000 | 102, 0x80000000 | 109, QMetaType::Bool, 0x80000000 | 111,   87,   99,  108,   86,  110,  112,
    QMetaType::Void, 0x80000000 | 90, 0x80000000 | 98, 0x80000000 | 102, 0x80000000 | 109, QMetaType::Bool,   87,   99,  108,   86,  110,
    QMetaType::Void, 0x80000000 | 90, 0x80000000 | 98, 0x80000000 | 102, 0x80000000 | 109,   87,   99,  108,   86,
    QMetaType::Void, 0x80000000 | 90, 0x80000000 | 98, 0x80000000 | 104, 0x80000000 | 106, QMetaType::Bool,   87,   99,  105,   86,  107,
    QMetaType::Void, 0x80000000 | 90, 0x80000000 | 98, 0x80000000 | 104, 0x80000000 | 106,   87,   99,  105,   86,
    QMetaType::Void, 0x80000000 | 90, 0x80000000 | 98, 0x80000000 | 102, QMetaType::QString, QMetaType::Bool, 0x80000000 | 111,   87,   99,  108,   92,  110,  112,
    QMetaType::Void, 0x80000000 | 90, 0x80000000 | 98, 0x80000000 | 102, QMetaType::QString, QMetaType::Bool,   87,   99,  108,   92,  110,
    QMetaType::Void, 0x80000000 | 90, 0x80000000 | 98, 0x80000000 | 102, QMetaType::QString,   87,   99,  108,   92,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void CGAL::QGLViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QGLViewer *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->viewerInitialized(); break;
        case 1: _t->drawNeeded(); break;
        case 2: _t->drawFinished((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->animateNeeded(); break;
        case 4: _t->helpRequired(); break;
        case 5: _t->axisIsDrawnChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->gridIsDrawnChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->FPSIsDisplayedChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->textIsEnabledChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->cameraIsEditedChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->pointSelected((*reinterpret_cast< const QMouseEvent*(*)>(_a[1]))); break;
        case 11: _t->mouseGrabberChanged((*reinterpret_cast< qglviewer::MouseGrabber*(*)>(_a[1]))); break;
        case 12: _t->contextIsDestroyed(); break;
        case 13: _t->needNewContext(); break;
        case 14: _t->setAxisIsDrawn((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 15: _t->setAxisIsDrawn(); break;
        case 16: _t->setGridIsDrawn((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 17: _t->setGridIsDrawn(); break;
        case 18: _t->setFPSIsDisplayed((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 19: _t->setFPSIsDisplayed(); break;
        case 20: _t->setTextIsEnabled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 21: _t->setTextIsEnabled(); break;
        case 22: _t->setCameraIsEdited((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 23: _t->setCameraIsEdited(); break;
        case 24: _t->toggleAxisIsDrawn(); break;
        case 25: _t->toggleGridIsDrawn(); break;
        case 26: _t->toggleFPSIsDisplayed(); break;
        case 27: _t->toggleTextIsEnabled(); break;
        case 28: _t->toggleCameraIsEdited(); break;
        case 29: _t->setBackgroundColor((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 30: _t->setForegroundColor((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 31: _t->setSceneRadius((*reinterpret_cast< qreal(*)>(_a[1]))); break;
        case 32: _t->setSceneCenter((*reinterpret_cast< const qglviewer::Vec(*)>(_a[1]))); break;
        case 33: _t->setSceneBoundingBox((*reinterpret_cast< const qglviewer::Vec(*)>(_a[1])),(*reinterpret_cast< const qglviewer::Vec(*)>(_a[2]))); break;
        case 34: _t->showEntireScene(); break;
        case 35: _t->setCamera((*reinterpret_cast< qglviewer::Camera*const(*)>(_a[1]))); break;
        case 36: _t->setManipulatedFrame((*reinterpret_cast< qglviewer::ManipulatedFrame*(*)>(_a[1]))); break;
        case 37: _t->setMouseGrabber((*reinterpret_cast< qglviewer::MouseGrabber*(*)>(_a[1]))); break;
        case 38: _t->setFullScreen((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 39: _t->setFullScreen(); break;
        case 40: _t->toggleFullScreen(); break;
        case 41: _t->toggleCameraMode(); break;
        case 42: _t->copyBufferToTexture((*reinterpret_cast< GLint(*)>(_a[1])),(*reinterpret_cast< GLenum(*)>(_a[2]))); break;
        case 43: _t->copyBufferToTexture((*reinterpret_cast< GLint(*)>(_a[1]))); break;
        case 44: _t->setAnimationPeriod((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 45: _t->startAnimation(); break;
        case 46: _t->stopAnimation(); break;
        case 47: _t->animate(); break;
        case 48: _t->toggleAnimation(); break;
        case 49: _t->help(); break;
        case 50: _t->aboutQGLViewer(); break;
        case 51: _t->select((*reinterpret_cast< const QMouseEvent*(*)>(_a[1]))); break;
        case 52: _t->select((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 53: _t->setSelectBufferSize((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 54: _t->setSelectRegionWidth((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 55: _t->setSelectRegionHeight((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 56: _t->setSelectedName((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 57: _t->setShortcut((*reinterpret_cast< qglviewer::KeyboardAction(*)>(_a[1])),(*reinterpret_cast< uint(*)>(_a[2]))); break;
        case 58: _t->setShortcut((*reinterpret_cast< qglviewer::KeyboardAction(*)>(_a[1])),(*reinterpret_cast< ::Qt::Modifier(*)>(_a[2])),(*reinterpret_cast< ::Qt::Key(*)>(_a[3]))); break;
        case 59: _t->setKeyDescription((*reinterpret_cast< uint(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 60: _t->setKeyDescription((*reinterpret_cast< ::Qt::KeyboardModifier(*)>(_a[1])),(*reinterpret_cast< ::Qt::Key(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 61: _t->setKeyDescription((*reinterpret_cast< ::Qt::Modifier(*)>(_a[1])),(*reinterpret_cast< ::Qt::Key(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 62: _t->clearShortcuts(); break;
        case 63: _t->setPathKey((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< uint(*)>(_a[2]))); break;
        case 64: _t->setPathKey((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 65: _t->setPlayPathKeyboardModifiers((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1]))); break;
        case 66: _t->setAddKeyFrameKeyboardModifiers((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1]))); break;
        case 67: _t->setMouseBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5]))); break;
        case 68: _t->setMouseBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[4]))); break;
        case 69: _t->setMouseBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4])),(*reinterpret_cast< ::Qt::MouseButtons(*)>(_a[5]))); break;
        case 70: _t->setMouseBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4]))); break;
        case 71: _t->setMouseBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[3]))); break;
        case 72: _t->setWheelBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4]))); break;
        case 73: _t->setWheelBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[3]))); break;
        case 74: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4])),(*reinterpret_cast< ::Qt::MouseButtons(*)>(_a[5]))); break;
        case 75: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4]))); break;
        case 76: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 77: _t->setMouseBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[4])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[5])),(*reinterpret_cast< bool(*)>(_a[6]))); break;
        case 78: _t->setMouseBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[4])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[5]))); break;
        case 79: _t->setMouseBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5])),(*reinterpret_cast< ::Qt::MouseButtons(*)>(_a[6]))); break;
        case 80: _t->setMouseBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5]))); break;
        case 81: _t->setMouseBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[4]))); break;
        case 82: _t->setWheelBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5]))); break;
        case 83: _t->setWheelBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[4]))); break;
        case 84: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5])),(*reinterpret_cast< ::Qt::MouseButtons(*)>(_a[6]))); break;
        case 85: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5]))); break;
        case 86: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 87: _t->clearMouseBindings(); break;
        case 88: _t->resetVisualHints(); break;
        case 89: _t->delayedFullScreen(); break;
        case 90: _t->hideMessage(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QGLViewer::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::viewerInitialized)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::drawNeeded)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::drawFinished)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::animateNeeded)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::helpRequired)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::axisIsDrawnChanged)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::gridIsDrawnChanged)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::FPSIsDisplayedChanged)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::textIsEnabledChanged)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::cameraIsEditedChanged)) {
                *result = 9;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)(const QMouseEvent * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::pointSelected)) {
                *result = 10;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)(qglviewer::MouseGrabber * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::mouseGrabberChanged)) {
                *result = 11;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::contextIsDestroyed)) {
                *result = 12;
                return;
            }
        }
        {
            using _t = void (QGLViewer::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLViewer::needNewContext)) {
                *result = 13;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CGAL::QGLViewer::staticMetaObject = { {
    QMetaObject::SuperData::link<QOpenGLWidget::staticMetaObject>(),
    qt_meta_stringdata_CGAL__QGLViewer.data,
    qt_meta_data_CGAL__QGLViewer,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CGAL::QGLViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CGAL::QGLViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CGAL__QGLViewer.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "QOpenGLFunctions"))
        return static_cast< QOpenGLFunctions*>(this);
    return QOpenGLWidget::qt_metacast(_clname);
}

int CGAL::QGLViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QOpenGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 91)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 91;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 91)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 91;
    }
    return _id;
}

// SIGNAL 0
void CGAL::QGLViewer::viewerInitialized()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void CGAL::QGLViewer::drawNeeded()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void CGAL::QGLViewer::drawFinished(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void CGAL::QGLViewer::animateNeeded()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void CGAL::QGLViewer::helpRequired()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}

// SIGNAL 5
void CGAL::QGLViewer::axisIsDrawnChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void CGAL::QGLViewer::gridIsDrawnChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void CGAL::QGLViewer::FPSIsDisplayedChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void CGAL::QGLViewer::textIsEnabledChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void CGAL::QGLViewer::cameraIsEditedChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void CGAL::QGLViewer::pointSelected(const QMouseEvent * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void CGAL::QGLViewer::mouseGrabberChanged(qglviewer::MouseGrabber * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void CGAL::QGLViewer::contextIsDestroyed()
{
    QMetaObject::activate(this, &staticMetaObject, 12, nullptr);
}

// SIGNAL 13
void CGAL::QGLViewer::needNewContext()
{
    QMetaObject::activate(this, &staticMetaObject, 13, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
