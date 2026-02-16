# Joystick2crsf

Joystick2crsf is a small SDL2 utility that maps joystick inputs to Crossfire (CRSF) channels and
streams them over UDP. It can also publish telemetry as a server-sent events stream, expose RC
channels through MAVLink when configured, and forward serial receiver output to a second UDP
destination with one framed protocol packet per UDP datagram.

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

The project uses standard Linux and C library headers. To build on a development machine:

```sh
set -euxo pipefail
sudo apt-get update
sudo apt-get install -y --no-install-recommends build-essential
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
define JOYSTICK2CRSF_BUILD_CMDS
$(TARGET_MAKE_ENV) $(MAKE) -C $(@D)
endef

define JOYSTICK2CRSF_INSTALL_TARGET_CMDS
$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) DESTDIR=$(TARGET_DIR) install
endef
```

The resulting root filesystem will include the binary in `/usr/bin`, the configuration under `/etc`,
and init files in the usual systemd and sysvinit locations.

If gamepad devices appear slowly during boot, set `startup_delay` in the configuration
(seconds, defaults to 5) to pause before the first device discovery; set to `0` to disable the
delay. Channels use CRSF scaling where 1811 is max and 172 is min.

Device and UDP reconnect attempts use `rescan_interval` seconds (default `10`), so if a joystick,
serial device, or UDP target is temporarily unavailable, the process keeps running and retries.
UDP sends are connectionless; if routing/peer availability flaps, packets are still attempted each
cycle and temporary send failures are dropped.

## Serial passthrough mode

Use the serial passthrough settings when your ELRS receiver already emits valid CRSF or MAVLink
frames on UART and you want those bytes wrapped directly into UDP datagrams:

```ini
serial_enabled=true
serial_device=/dev/ttyS4
serial_baud=420000
serial_udp_enabled=true
serial_udp_target=192.168.2.10:14551
serial_packetizer=crsf
```

`serial_packetizer` controls framing for strict packet boundaries:

- `crsf` (default): validates CRSF CRC and sends one CRSF frame per UDP packet.
- `mavlink`: sends one MAVLink frame (v1/v2) per UDP packet using MAVLink framing bytes.
- `raw`: forwards raw read chunks without packetizing.

You can run joystick output and serial passthrough at the same time with different UDP targets
(for example `udp_target=...:14550` and `serial_udp_target=...:14551`).
When SSE is enabled, joystick and serial data are published as separate SSE events
(`event: joystick` and `event: serial`) with a `stream` field in the JSON payload.
SSE output is throttled to `10 Hz` by default and emits the latest available frame per stream.
Use `sse_rate_hz` in the config to change the SSE update rate (`1` to `100`).

Note: action-key hooks have been removed. Existing `action_*` and related action
configuration lines are ignored by current builds and should be deleted from local configs.
