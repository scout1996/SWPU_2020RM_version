TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++11

SOURCES += \
    Main/main.cpp \
    Main/threadcontrol.cpp \
    Armor/armordetector.cpp \
    General/opencv_extended.cpp

INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
               /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_imgcodecs.so \
        /usr/local/lib/libopencv_video.so \
        /usr/local/lib/libopencv_videoio.so \
        /usr/local/lib/libopencv_calib3d.so \
        /usr/local/lib/libopencv_ml.so \
        /usr/lib/x86_64-linux-gnu/libpthread.so

HEADERS += \
    Main/threadcontrol.h \
    Armor/armordetector.h \
    General/opencv_extended.h
