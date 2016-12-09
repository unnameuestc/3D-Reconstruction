# OpenCV ----------------------------------------------------------------------------------

# Windows 7
win32:OPENCV_DIR = "D:/OpenCV/build"
win32:OPENCV_LIB_DIR = $$OPENCV_DIR/x86/vc10/lib
win32:CV_VER = 246

CV_LIB_NAMES = core imgproc highgui calib3d features2d flann nonfree contrib
for(lib, CV_LIB_NAMES) {
    CV_LIBS += -lopencv_$$lib
}

win32 {
    DEFINES += NOMINMAX _CRT_SECURE_NO_WARNINGS _SCL_SECURE_NO_WARNINGS _USE_MATH_DEFINES
    QMAKE_CXXFLAGS_WARN_ON += -W3 -wd4396 -wd4100 -wd4996
    QMAKE_LFLAGS += /INCREMENTAL:NO
    DSHOW_LIBS = -lStrmiids -lVfw32 -lOle32 -lOleAut32

    CONFIG(release, debug|release) {
        CV_LIB_PREFIX = $$CV_VER
    }
    else {
        CV_LIB_PREFIX = $${CV_VER}d
    }
    for(lib, CV_LIBS) {
        CV_LIBS_NEW += $$lib$$CV_LIB_PREFIX
    }
    CV_LIBS = $$CV_LIBS_NEW $$CV_EXT_LIBS $$DSHOW_LIBS
}

LIBS += -L$$OPENCV_LIB_DIR $$CV_LIBS
INCLUDEPATH += $$OPENCV_DIR/include
