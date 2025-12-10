# Joystick2crsf

Joystick2crsf is a small SDL2 utility that maps joystick inputs to Crossfire (CRSF) channels and
streams them over UDP. It can also publish telemetry as a server-sent events stream and expose RC
channels through MAVLink when configured.

## What is in this repository

- `joystick2crfs.c`: the main application source.
- `joystick2crfs.conf`: default configuration file read by the binary.
- `S96joystick2crsf`: a simple init script for sysvinit-based systems.
- `joystick2crsf.service`: a systemd unit with placeholder paths.
- `Makefile`: a Buildroot-friendly build script that builds and installs the binary and helpers.

## Building

The project depends on SDL2 headers and `pkg-config`. To build on a development machine:

```sh
make
```

You can override toolchain variables if you are cross-compiling:

```sh
make CC=aarch64-linux-gnu-gcc PKG_CONFIG="/path/to/target-pkg-config"
```

## Installing

The install target is ready for staged installs with `DESTDIR`, which aligns with Buildroot's
`generic-package` flow:

```sh
make DESTDIR=/tmp/rootfs install
```

This copies the binary to `/usr/bin/joystick2crsf`, installs the default configuration under
`/etc/joystick2crsf.conf`, and places both the sysvinit script and the systemd unit. During install
the systemd unit placeholders are rewritten so that `ExecStart` points at the installed binary and
configuration path.

## Using with Buildroot (BR2)

Add this repository as an external tree or vendor package source, then create a Buildroot package
that sets `JOYSTICK2CRSF_SITE_METHOD = local` and `JOYSTICK2CRSF_SITE` to the location of this
checkout. In your package `.mk` file, call the build and install targets with Buildroot's
`TARGET_MAKE_ENV`:

```make
JOYSTICK2CRSF_DEPENDENCIES = sdl2

define JOYSTICK2CRSF_BUILD_CMDS
$(TARGET_MAKE_ENV) $(MAKE) -C $(@D)
endef

define JOYSTICK2CRSF_INSTALL_TARGET_CMDS
$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) DESTDIR=$(TARGET_DIR) install
endef
```

The resulting root filesystem will include the binary in `/usr/bin`, the configuration under `/etc`,
and init files in the usual systemd and sysvinit locations.
