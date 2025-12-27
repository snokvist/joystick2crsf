TARGET ?= joystick2crsf
TARGET_KEYS ?= action_keys
SRCS := joystick2crsf.c
SRCS_KEYS := action_keys.c
OBJS := $(SRCS:.c=.o)
OBJS_KEYS := $(SRCS_KEYS:.c=.o)

PREFIX ?= /usr
BINDIR ?= $(PREFIX)/bin
CONFDIR ?= /etc
INITDDIR ?= /etc/init.d
SYSTEMD_UNITDIR ?= /usr/lib/systemd/system

JOYSTICK2CRSF_BIN ?= $(BINDIR)/$(TARGET)
JOYSTICK2CRSF_CONF ?= $(CONFDIR)/joystick2crsf.conf

PKG_CONFIG ?= pkg-config
SDL2_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags sdl2)
SDL2_LIBS ?= $(shell $(PKG_CONFIG) --libs sdl2)
CURL_CFLAGS ?=
CURL_LIBS ?=

CFLAGS ?= -O2 -g -Wall -Wextra
LDFLAGS ?=
LIBS := $(SDL2_LIBS)
LIBS_KEYS :=

INSTALL ?= install
DESTDIR ?=

all: $(TARGET) $(TARGET_KEYS)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

$(TARGET_KEYS): $(OBJS_KEYS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS_KEYS)

%.o: %.c
	$(CC) $(CFLAGS) $(SDL2_CFLAGS) -c -o $@ $<

action_keys.o: action_keys.c
	$(CC) $(CFLAGS) $(CURL_CFLAGS) -c -o $@ $<

install: $(TARGET) $(TARGET_KEYS)
	$(INSTALL) -D -m 0755 $(TARGET) $(DESTDIR)$(JOYSTICK2CRSF_BIN)
	$(INSTALL) -D -m 0644 joystick2crsf.conf \
		$(DESTDIR)$(JOYSTICK2CRSF_CONF)
	$(INSTALL) -D -m 0755 $(TARGET_KEYS) \
		$(DESTDIR)$(BINDIR)/$(TARGET_KEYS)
	$(INSTALL) -D -m 0755 S96joystick2crsf \
		$(DESTDIR)$(INITDDIR)/S96joystick2crsf
	$(INSTALL) -D -m 0644 joystick2crsf.service \
		$(DESTDIR)$(SYSTEMD_UNITDIR)/joystick2crsf.service
	sed -i \
		-e 's#@JOYSTICK2CRSF_BIN@#$(JOYSTICK2CRSF_BIN)#g' \
		-e 's#@JOYSTICK2CRSF_CONF@#$(JOYSTICK2CRSF_CONF)#g' \
		$(DESTDIR)$(SYSTEMD_UNITDIR)/joystick2crsf.service

clean:
	rm -f $(TARGET) $(TARGET_KEYS) $(OBJS) $(OBJS_KEYS)

.PHONY: all install clean
