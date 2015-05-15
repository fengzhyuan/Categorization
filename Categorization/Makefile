

CXX = g++
CC = gcc

CFLAGS = -c -w -O2 -I./include/ -I/usr/local/include/opencv -I/usr/local/include -I/usr/include/gtk-2.0/ -I/usr/lib/i386-linux-gnu/gtk-2.0/include  -I/usr/include/atk-1.0 -I/usr/include/cairo -I/usr/include/gdk-pixbuf-2.0 -I/usr/include/pango-1.0 -I/usr/include/gio-unix-2.0/ -I/usr/include/glib-2.0 -I/usr/lib/i386-linux-gnu/glib-2.0/include -I/usr/include/pixman-1 -I/usr/include/freetype2  -I/usr/include/libpng12 -ffloat-store

CXXFLAGS = -c -w -O2 -D__STDC_LIMIT_MACROS -I. -I./include/ -I/usr/local/include/opencv -I/usr/local/include -I/usr/include/gtk-2.0/ -I/usr/lib/i386-linux-gnu/gtk-2.0/include  -I/usr/include/atk-1.0 -I/usr/include/cairo -I/usr/include/gdk-pixbuf-2.0 -I/usr/include/pango-1.0 -I/usr/include/gio-unix-2.0/ -I/usr/include/glib-2.0 -I/usr/lib/i386-linux-gnu/glib-2.0/include -I/usr/include/pixman-1 -I/usr/include/freetype2  -I/usr/include/libpng12 -ffloat-store -L/scratch/software/openCV/OpenCV-2.4.3/lib -L.

LIBS = -lgsl -lgslcblas -lpthread 

OUTPUTDIR = ./bin/
MKDIR = mkdir -p $(OUTPUTDIR)
SRC = ./src/
	
CXXSOURCES = \
    $(SRC)*.cpp 

all:detector

detector: 
	$(MKDIR) 
	$(CXX) $(CXXFLAGS) $(CXXSOURCES) 
	$(CXX) *.o  $(LIBS) `pkg-config --libs opencv` -o$(OUTPUTDIR)detector
	rm -rf *.o

clean:
	rm -rf $(OUTPUTDIR)
	rm -rf ./*.o


