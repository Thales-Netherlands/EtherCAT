lib_LTLIBRARIES = libethercattty.la

libethercattty_la_SOURCES = \
	tty.c \
    cdev.c

noinst_HEADERS = \
	tty.h \
    cdev.h \
	ioctl.h 

libethercattty_la_CFLAGS = -fno-strict-aliasing -Wall -I$(top_srcdir)
libethercattty_la_LDFLAGS = -version-info 1:0:0




