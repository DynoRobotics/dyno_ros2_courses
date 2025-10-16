# Simple Makefile for ros2_zenoh_c library
# Suitable for Buildroot integration

CC = gcc
CFLAGS = -std=c99 -Wall -Wextra -O2 -fPIC
INCLUDES = -Iinclude -I/opt/ros/jazzy/opt/zenoh_cpp_vendor/include -I/home/ubuntu/ws/install/micro-cdr/microcdr-2.0.2/include
LDFLAGS = -L/opt/ros/jazzy/opt/zenoh_cpp_vendor/lib -L/home/ubuntu/ws/install/micro-cdr/microcdr-2.0.2/lib
LIBS = -lzenohc -lmicrocdr

# Define macros for optional features
CFLAGS += -DZENOH_AVAILABLE -DMICROCDR_AVAILABLE

# Source files
SRCDIR = src
INCDIR = include
OBJDIR = obj
LIBDIR = lib
BINDIR = bin

SOURCES = $(SRCDIR)/ros2_zenoh_c_simple.c $(SRCDIR)/ros2_messages.c $(SRCDIR)/ros2_zenoh_c_stubs.c
OBJECTS = $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)
TARGET_LIB = $(LIBDIR)/libros2_zenoh_c.a
TARGET_SO = $(LIBDIR)/libros2_zenoh_c.so

# Example executables
EXAMPLES = $(BINDIR)/publisher_example $(BINDIR)/subscriber_example $(BINDIR)/publisher_example_zenoh $(BINDIR)/subscriber_example_zenoh

.PHONY: all clean install examples

all: $(TARGET_LIB) $(TARGET_SO)

$(OBJDIR)/%.o: $(SRCDIR)/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(TARGET_LIB): $(OBJECTS) | $(LIBDIR)
	ar rcs $@ $^

$(TARGET_SO): $(OBJECTS) | $(LIBDIR)
	$(CC) -shared -o $@ $^ $(LDFLAGS) $(LIBS)

examples: $(EXAMPLES)

$(BINDIR)/publisher_example: examples/publisher_example.c $(TARGET_LIB) | $(BINDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $< $(TARGET_LIB) $(LDFLAGS) $(LIBS)

$(BINDIR)/subscriber_example: examples/subscriber_example.c $(TARGET_LIB) | $(BINDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $< $(TARGET_LIB) $(LDFLAGS) $(LIBS)

$(BINDIR)/publisher_example_zenoh: examples/publisher_example_zenoh.c | $(BINDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $< $(LDFLAGS) $(LIBS)

$(BINDIR)/subscriber_example_zenoh: examples/subscriber_example_zenoh.c | $(BINDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $< $(LDFLAGS) $(LIBS)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(LIBDIR):
	mkdir -p $(LIBDIR)

$(BINDIR):
	mkdir -p $(BINDIR)

install: $(TARGET_LIB) $(TARGET_SO)
	mkdir -p $(DESTDIR)/usr/lib
	mkdir -p $(DESTDIR)/usr/include/ros2_zenoh_c
	cp $(TARGET_LIB) $(TARGET_SO) $(DESTDIR)/usr/lib/
	cp $(INCDIR)/ros2_zenoh_c/*.h $(DESTDIR)/usr/include/ros2_zenoh_c/

clean:
	rm -rf $(OBJDIR) $(LIBDIR) $(BINDIR)

# For Buildroot integration
buildroot-install: install
