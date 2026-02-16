TARGET ?= joystick2crsf
SRCS := joystick2crsf.c
OBJS := $(SRCS:.c=.o)

PREFIX ?= /usr
BINDIR ?= $(PREFIX)/bin
CONFDIR ?= /etc
INITDDIR ?= /etc/init.d
SYSTEMD_UNITDIR ?= /usr/lib/systemd/system

JOYSTICK2CRSF_BIN ?= $(BINDIR)/$(TARGET)
JOYSTICK2CRSF_CONF ?= $(CONFDIR)/joystick2crsf.conf

CFLAGS ?= -O2 -g -Wall -Wextra -pthread
LDFLAGS ?= -pthread

INSTALL ?= install
DESTDIR ?=

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

install: $(TARGET)
	$(INSTALL) -D -m 0755 $(TARGET) $(DESTDIR)$(JOYSTICK2CRSF_BIN)
	$(INSTALL) -D -m 0644 joystick2crsf.conf \
		$(DESTDIR)$(JOYSTICK2CRSF_CONF)
	$(INSTALL) -D -m 0755 S96joystick2crsf \
		$(DESTDIR)$(INITDDIR)/S96joystick2crsf
	$(INSTALL) -D -m 0644 joystick2crsf.service \
		$(DESTDIR)$(SYSTEMD_UNITDIR)/joystick2crsf.service
	sed -i \
		-e 's#@JOYSTICK2CRSF_BIN@#$(JOYSTICK2CRSF_BIN)#g' \
		-e 's#@JOYSTICK2CRSF_CONF@#$(JOYSTICK2CRSF_CONF)#g' \
		$(DESTDIR)$(SYSTEMD_UNITDIR)/joystick2crsf.service

clean:
	rm -f $(TARGET) $(OBJS)

.PHONY: all install clean
