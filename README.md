# Joystick2crsf

Joystick2crsf is a small SDL2 utility that maps joystick inputs to Crossfire (CRSF) channels and
streams them over UDP. It can also publish telemetry as a server-sent events stream and expose RC
channels through MAVLink when configured.

<img
  width="1920"
  height="1080"
  alt="image"
  src="https://github.com/user-attachments/assets/162c15f7-df07-4a0d-b02b-1f4b3440a9f9"
/>


## What is in this repository

- `joystick2crsf.c`: the main application source.
- `joystick2crsf.conf`: default configuration file read by the binary.
- `S96joystick2crsf`: a simple init script for sysvinit-based systems.
- `joystick2crsf.service`: a systemd unit with placeholder paths.
- `Makefile`: a Buildroot-friendly build script that builds and installs the binary and helpers.

## Building

The project depends on SDL2 headers, libcurl (for `action_keys`), and `pkg-config`. To build on a
development machine:

```sh
set -euxo pipefail
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential pkg-config libsdl2-dev libcurl4-openssl-dev
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

## Keyboard bindings

Optional keyboard bindings let a channel emit synthetic keypresses through `/dev/uinput`. Short and
long presses map to `key_short_N`/`key_long_N` (or the `_low` variants for negative edges). Values
are trimmed of whitespace before parsing and accept `up`, `down`, `left`, `right`, `enter`/`return`,
`space`, `a`–`z`, and `0`–`9` (case-insensitive). Ensure the runtime user can open `/dev/uinput`
which may require root or membership in the `uinput` group.

If gamepad devices appear slowly during boot, set `startup_delay` in the configuration (seconds,
defaults to 5) to pause before the first device discovery; set to `0` to disable the delay.

Channels use CRSF scaling where 1811 is max and 172 is min. High-edge presses arm at 1700 and
release at 1500, leaving a 200-count hysteresis. Low-edge presses mirror that around the CRSF
minimum: they trigger at 283 (1811 symmetrical to 1700) and release at 483, keeping the same gap
to avoid chattering while the stick settles.

## Action keys configuration

`joystick2crsf` reads `/etc/action_keys.conf` when it is readable and fires UDP or HTTP actions
whenever channel-based key bindings trigger. You can keep actions in that separate file while the
runtime drives them directly, avoiding the extra input-polling helper.

Key features:

- Bind actions to logical keys (`key=...`) referenced by `key_short_N`/`key_long_N` (and `_low`
  variants). Names `up`, `down`, `left`, `right`, `enter`/`return`, `space`/`spacebar`, `a`–`z`,
  and `0`–`9` are supported.
- Transports: `udp` (send payload verbatim) or `http` (`GET`/`POST` with optional headers).
- Config fields: `destination`/`url`, `body`, `header`, and `timeout_ms`.

Example config (`/etc/action_keys.conf`):

```
action_1=key=a,transport=udp,url=udp://10.0.0.2:9000,body={"ping":1}
action_2=key=b,transport=http,url=http://10.0.0.2/api,method=POST,body={"cmd":"start"}
```

The dispatch runs even when `/dev/uinput` is unavailable, so remove or rename the config file if you
want to disable the action hooks.
