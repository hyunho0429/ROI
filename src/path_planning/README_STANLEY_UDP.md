# MORAI UDP Stanley controller

This controller follows the ENU CSV produced by `morai_global_csv_recorder.py`.
It is a standalone UDP program; ROS and a catkin workspace are not required at
runtime.

## Why the old modules were not copied

The supplied `alignment.py`, `ekf.py`, `mechanization.py`, and
`system_dynamics.py` form a physical OpenIMU/u-blox strapdown INS stack. They
subscribe to ROS messages and integrate latitude/longitude, Earth rotation,
gravity, and curvature in an NED frame. That implementation cannot be reused
unchanged with MORAI UDP.

Because the competition adds GPS and IMU noise, localization itself must not be
removed. This branch replaces the old stack with a small planar EKF whose state
is `[x, y, vx, vy, yaw, gyro_z_bias]`. It fuses:

- GPS position and NMEA ground speed/course
- IMU quaternion yaw and yaw angular velocity

Stanley only needs horizontal position, heading, and speed. GPS altitude is
low-pass filtered and retained for 3-D path-segment disambiguation; it is not
used as a condition that must change before a waypoint can be followed. IMU
linear acceleration is parsed but deliberately not integrated: on a slope it
contains gravity/projection effects, and integrating it without a fully
calibrated sensor model creates more drift than it removes.

`--localization ego` remains available only for network/controller debugging.
It uses simulator Ground Truth and must not be used for a noisy-sensor
competition run.

## Coordinates confirmed from MORAI documentation

- The [sensor protocol](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/-35)
  says GPS UDP follows NMEA0183 and simultaneously sends RMC/GGA sentences with
  latitude, longitude, and altitude. Therefore GPS UDP is **not a UTM x/y wire
  packet**.
- The [sensor coordinate page](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/-8)
  specifies GPS as WGS84 UTM zone 52N (EPSG:32652), and IMU axes as x-forward,
  y-left, z-up with counter-clockwise positive yaw.
- The [24.R1 MapSpec definition](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R1.0/ros-1)
  identifies `UTMoffset` as the map offset. The UDP sensor protocol does not
  define a MapSpec service packet, so this program reads the equivalent MGeo
  `global_info.json` (`global_coordinate_system` and
  `local_origin_in_global`).
- The same 24.R1 protocol page defines the exact 55-byte `Ego Ctrl Cmd` packet,
  including normalized steering, and the 181-byte `Ego Vehicle Status` packet.
- MORAI documents GPS Gaussian noise/blackout and IMU Gaussian noise,
  bias-instability, random-walk, and blackout in the
  [Denied Area page](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R1.0/denied-area).

Important version boundary: the official sensor-protocol page marks the
107-byte IMU UDP packet as a **24.R2.2 update**. The 24.R1 protocol page does
not document an IMU UDP packet. If the competition simulator is truly 24.R1
and does not emit `#IMUData$` packets, this parser cannot invent that missing
protocol; use the interface/version supplied by the organizer or obtain their
packet specification. Ego control and Ego status in this branch are strictly
24.R1 packet layouts.

## Install

Python 3.8 or newer is recommended. Only the GPS projection dependency is
external.

```bash
python3 -m pip install -r src/path_planning/requirements.txt
```

## MORAI network settings

Example values for one PC:

| MORAI item | Direction | IP | Port |
|---|---|---:|---:|
| GPS sensor UDP | SIM to program | `127.0.0.1` | `9100` |
| IMU sensor UDP | SIM to program | `127.0.0.1` | `9101` |
| Ego Ctrl Cmd | program to SIM | `127.0.0.1` | `9090` |

The port numbers are examples; MORAI and the command-line options must match.
Place the Ego controller in external/Auto control mode before running. Do not
assign the same receive port to multiple sensor messages.

## Run

From the repository root:

```bash
python3 src/path_planning/src/morai_stanley_udp.py \
  --path src/path_planning/data/morai_global_path.csv \
  --gps-port 9100 \
  --imu-port 9101 \
  --control-ip 127.0.0.1 \
  --control-port 9090 \
  --target-speed-kmh 20
```

For a map other than the included K-City 2025 map, pass its
`mgeo/.../global_info.json`:

```bash
python3 src/path_planning/src/morai_stanley_udp.py \
  --path path.csv \
  --global-info /absolute/path/to/global_info.json
```

The program brakes when GPS/IMU becomes stale, when localization is not yet
initialized, at the end of the path, and on `Ctrl+C`.
MORAI's documented all-zero GPS blackout is rejected, as are GPS position jumps
larger than 15 m from the predicted state; rejected fixes do not refresh the
watchdog.

## Parameters that must be verified on the actual vehicle

- `--max-steering-deg`: maximum front-wheel angle used to normalize the MORAI
  command. The default `36.25` is a starting value, not a universal vehicle
  specification.
- `--morai-steer-sign`: defaults to `-1`, converting mathematical left-positive
  Stanley angle to MORAI command convention. Verify at very low speed; use `1`
  if the selected controller/model has the opposite convention.
- `--control-point-offset`: defaults to `0`. MORAI 24.R1 describes `posXYZ` only
  as vehicle position and does not identify front axle, rear axle, or center of
  mass. The recorded path uses the same reported reference. Set a forward
  offset only after the competition vehicle's reference point is confirmed.
- `--imu-yaw-offset-deg` and `--imu-yaw-sign`: compensate only for a known
  sensor mounting rotation/convention. With a forward-aligned documented ENU
  IMU, keep `0` and `1`.

Start at 5-10 km/h and tune `--stanley-gain`, then increase speed. A larger gain
corrects cross-track error more aggressively but can oscillate with noisy GPS.
