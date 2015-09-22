#-------------------------------------------------
#
# Project created by QtCreator 2013-02-25T17:13:11
#
#-------------------------------------------------

QT       += core gui

TARGET = annotation_tool
TEMPLATE = app


SOURCES += src/main.cpp\
	src/mainwindow.cpp \
	src/pointcloudmodifier.cpp \
    src/viewerinteractor.cpp \
    src/newobjectdialog.cpp \
    src/objectsinformation.cpp \
    src/selectobjectdialog.cpp \
    src/initialmessagedialog.cpp \
    src/filtervaluesdialog.cpp \
    src/addobjectdialog.cpp


HEADERS  += src/mainwindow.h \
	 src/pointcloudmodifier.h \
    src/viewerinteractor.h \
    src/newobjectdialog.h \
    src/objectsinformation.h \
    src/selectobjectdialog.h \
    src/initialmessagedialog.h \
    src/filtervaluesdialog.h \
    src/addobjectdialog.h


FORMS    += src/mainwindow.ui \
    src/newobjectdialog.ui \
    src/selectobjectdialog.ui \
    src/initialmessagedialog.ui \
    src/filtervaluesdialog.ui \
    src/addobjectdialog.ui

INCLUDEPATH +=  "/opt/ros/fuerte/include/pcl-1.5" \
		"/usr/local/include/opencv" \
		"/usr/include/pcl-1.6" \
		#"/home/adria/openCV/opencv/include/opencv/" \
		"/usr/include/vtk-5.8" \

RESOURCES += \
    icons/icons.qrc

OTHER_FILES +=



