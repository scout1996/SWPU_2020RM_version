#############################################################################
# Makefile for building: SWPU_2020RoboMaster_version
# Generated by qmake (2.01a) (Qt 4.8.7) on: ?? 9? 30 17:52:04 2019
# Project:  SWPU_2020RoboMaster_version.pro
# Template: app
# Command: /usr/lib/x86_64-linux-gnu/qt4/bin/qmake -spec /usr/share/qt4/mkspecs/linux-g++-64 CONFIG+=debug -o Makefile SWPU_2020RoboMaster_version.pro
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = 
CFLAGS        = -m64 -pipe -g -Wall -W $(DEFINES)
CXXFLAGS      = -m64 -pipe -std=c++11 -g -Wall -W $(DEFINES)
INCPATH       = -I/usr/share/qt4/mkspecs/linux-g++-64 -I. -I/usr/local/include -I/usr/local/include/opencv -I/usr/local/include/opencv2
LINK          = g++
LFLAGS        = -m64
LIBS          = $(SUBLIBS)   /usr/local/lib/libopencv_imgproc.so /usr/local/lib/libopencv_highgui.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_imgcodecs.so /usr/local/lib/libopencv_video.so /usr/local/lib/libopencv_videoio.so /usr/local/lib/libopencv_calib3d.so /usr/local/lib/libopencv_ml.so /usr/lib/x86_64-linux-gnu/libpthread.so 
AR            = ar cqs
RANLIB        = 
QMAKE         = /usr/lib/x86_64-linux-gnu/qt4/bin/qmake
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = Main/main.cpp \
		Main/threadcontrol.cpp \
		Armor/armordetector.cpp \
		General/opencv_extended.cpp \
		Pose/AngleSolver.cpp 
OBJECTS       = main.o \
		threadcontrol.o \
		armordetector.o \
		opencv_extended.o \
		AngleSolver.o
DIST          = camera_param/camera.xml \
		/usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/debug.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/shared.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		SWPU_2020RoboMaster_version.pro
QMAKE_TARGET  = SWPU_2020RoboMaster_version
DESTDIR       = 
TARGET        = SWPU_2020RoboMaster_version

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)
	{ test -n "$(DESTDIR)" && DESTDIR="$(DESTDIR)" || DESTDIR=.; } && test $$(gdb --version | sed -e 's,[^0-9][^0-9]*\([0-9]\)\.\([0-9]\).*,\1\2,;q') -gt 72 && gdb --nx --batch --quiet -ex 'set confirm off' -ex "save gdb-index $$DESTDIR" -ex quit '$(TARGET)' && test -f $(TARGET).gdb-index && objcopy --add-section '.gdb_index=$(TARGET).gdb-index' --set-section-flags '.gdb_index=readonly' '$(TARGET)' '$(TARGET)' && rm -f $(TARGET).gdb-index || true

Makefile: SWPU_2020RoboMaster_version.pro  /usr/share/qt4/mkspecs/linux-g++-64/qmake.conf /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/debug.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/shared.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf
	$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++-64 CONFIG+=debug -o Makefile SWPU_2020RoboMaster_version.pro
/usr/share/qt4/mkspecs/common/unix.conf:
/usr/share/qt4/mkspecs/common/linux.conf:
/usr/share/qt4/mkspecs/common/gcc-base.conf:
/usr/share/qt4/mkspecs/common/gcc-base-unix.conf:
/usr/share/qt4/mkspecs/common/g++-base.conf:
/usr/share/qt4/mkspecs/common/g++-unix.conf:
/usr/share/qt4/mkspecs/qconfig.pri:
/usr/share/qt4/mkspecs/features/qt_functions.prf:
/usr/share/qt4/mkspecs/features/qt_config.prf:
/usr/share/qt4/mkspecs/features/exclusive_builds.prf:
/usr/share/qt4/mkspecs/features/default_pre.prf:
/usr/share/qt4/mkspecs/features/debug.prf:
/usr/share/qt4/mkspecs/features/default_post.prf:
/usr/share/qt4/mkspecs/features/shared.prf:
/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf:
/usr/share/qt4/mkspecs/features/warn_on.prf:
/usr/share/qt4/mkspecs/features/resources.prf:
/usr/share/qt4/mkspecs/features/uic.prf:
/usr/share/qt4/mkspecs/features/yacc.prf:
/usr/share/qt4/mkspecs/features/lex.prf:
/usr/share/qt4/mkspecs/features/include_source_dir.prf:
qmake:  FORCE
	@$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++-64 CONFIG+=debug -o Makefile SWPU_2020RoboMaster_version.pro

dist: 
	@$(CHK_DIR_EXISTS) .tmp/SWPU_2020RoboMaster_version1.0.0 || $(MKDIR) .tmp/SWPU_2020RoboMaster_version1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) .tmp/SWPU_2020RoboMaster_version1.0.0/ && (cd `dirname .tmp/SWPU_2020RoboMaster_version1.0.0` && $(TAR) SWPU_2020RoboMaster_version1.0.0.tar SWPU_2020RoboMaster_version1.0.0 && $(COMPRESS) SWPU_2020RoboMaster_version1.0.0.tar) && $(MOVE) `dirname .tmp/SWPU_2020RoboMaster_version1.0.0`/SWPU_2020RoboMaster_version1.0.0.tar.gz . && $(DEL_FILE) -r .tmp/SWPU_2020RoboMaster_version1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


check: first

compiler_rcc_make_all:
compiler_rcc_clean:
compiler_uic_make_all:
compiler_uic_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: 

####### Compile

main.o: Main/main.cpp Main/threadcontrol.h \
		Armor/armordetector.h \
		Pose/AngleSolver.hpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o main.o Main/main.cpp

threadcontrol.o: Main/threadcontrol.cpp Main/threadcontrol.h \
		Armor/armordetector.h \
		Pose/AngleSolver.hpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o threadcontrol.o Main/threadcontrol.cpp

armordetector.o: Armor/armordetector.cpp Armor/armordetector.h \
		General/opencv_extended.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o armordetector.o Armor/armordetector.cpp

opencv_extended.o: General/opencv_extended.cpp General/opencv_extended.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o opencv_extended.o General/opencv_extended.cpp

AngleSolver.o: Pose/AngleSolver.cpp Pose/AngleSolver.hpp \
		Armor/armordetector.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o AngleSolver.o Pose/AngleSolver.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

