# AgroBot — Integrated Developer Reference

> Analyzed & verified 2026-07-11. ROS2 (`kilted`) agricultural robot on a Raspberry Pi.
> Mission intent: drive crop rows (**NAVIGATION**), see plants/weeds/seeds (**VISION**),
> and physically remove weeds / dispense seeds (**ACTUATION**), with 2× RoboClaw/BasicMicro
> motor controllers over UART.

> **Read this first — the big picture:** the three subsystems are only loosely coupled and the
> mission is **not** a closed autonomy loop today. Vision only counts objects and updates an LCD;
> its computed plant geometry is never published. Removal is a hardware IR-beam reflex that never
> consults the camera. Seeding is manual-button-only. Navigation is timer-scripted and ignores its
> ToF sensors. Treat each subsystem as an island that happens to share a Pi.

---

## 1. Workspace layout (`~/ros2_ws/src`)

| Package | Build type | Role | Depends on `agrobot_interfaces`? |
|---|---|---|---|
| `agrobot_interfaces` | ament_cmake (rosidl) | Defines `srv/MoveServo` | n/a |
| `agrobot_nav` | ament_python | Navigation + drive + camera + teleop | **No** (declares `tutorial_interfaces` instead) |
| `agrobot_detect` | ament_python | Vision + beam-break + LCD HMI | No |
| `agrobot_actuation` | ament_python | Remove/seed servos + triggers | **Yes** (only pkg that does) |
| `basicmicro_ros2` | ament_python | **Vendored** RoboClaw ros2_control driver (reference; mostly bypassed) | No |
| `tutorial_interfaces` | ament_cmake | Provides `msg/Num` used as the drive command | n/a |
| `examples` | — | ROS2 tutorial leftovers | n/a |

---

## 2. ROS graph — topics

All topic names verified by grep; there are **no near-miss/typo topic mismatches** — the risk areas
are dangling endpoints and duplicate producers.

| Topic | Type | Publisher(s) | Subscriber(s) | Notes |
|---|---|---|---|---|
| `lcd_display/text` | `std_msgs/String` | `vision_application_publisher`, `vision_lcd_publisher`, `lcd_display_publisher` | `lcd_display_subscriber` | 3 possible producers; run exactly one |
| `lcd_display/color` | `std_msgs/ColorRGBA` | same 3 | `lcd_display_subscriber` | color 0–1 range, alpha dropped |
| `/beam_break/blocked` | `std_msgs/Bool` | `beam_break_node` | `beam_break_remove` | True = beam broken; 20 Hz continuous |
| `tof_distance_0..7` | `std_msgs/Float32` | `tof_sensor_node` (node `tmf8801_tof_sensor`) | `nav_state_machine` | **stored but never used** by nav |
| `topic` | `tutorial_interfaces/msg/Num` | `input` (`minimal_publisher`) | `drive_control` (`minimal_subscriber`) **and** `nav_state_machine` | shared command bus, no arbitration |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `camera_node` | **none** | dangling; only the MJPEG stream consumes frames |
| `cmd_vel`, `odom`, `joint_states`, `diagnostics` | Twist/nav/sensor/diag | `basicmicro_node` (vendored, not launched) | none | vendored driver only |

**LCD producer choice:** run exactly one of the three producers. `vision_application_publisher` is the
launchable duplicate; `vision_lcd_publisher` is the "clean" library-driven producer but is **missing
from `setup.py`**; `lcd_display_publisher` is a demo that cycles canned strings.

---

## 3. ROS graph — services

The only custom service is `agrobot_interfaces/srv/MoveServo` = `string request → string response`.
**Both servers ignore `request.request`** (`# Ignore request.request intentionally`), so intent is
conveyed *only by which service name is called*. Response strings: `"OK: moved …"`,
`"BUSY: motion already in progress"`, `"ERROR: {e}"`.

| Service | Type | Server | Client(s) | Trigger |
|---|---|---|---|---|
| `remove_servo` | `MoveServo` | `remove.py` (`remove_servo_server`) | `beam_break_remove` (`'go'`), `button_client` (`'remove'`) | IR beam edge, or manual button |
| `seed_servo` | `MoveServo` | `seed.py` (`seed_servo_server`) | `button_client` (`'seed'`) | **manual button only** |

> `beam_break_remove` creates its client against `'/remove_servo'` (correct) but **logs**
> `Waiting for /beam_break_remove service...` — a nonexistent name. Misleading, harmless.

---

## 4. Node catalog

### Navigation (`agrobot_nav`)
- **`tof_sensor_node`** (exe `tof_sensor`, node `tmf8801_tof_sensor`) — 8× TMF8801 ToF over a TCA9548A
  I2C mux (0x70, bus 1). Raw register-level `smbus2`. Publishes `tof_distance_0..7` at 10 Hz. No
  calibration, permissive confidence gate, ~13 s blocking startup.
- **`nav_state_machine`** (exe `nav`) — 5-state FSM
  `IDLE → INITIAL_BURST → TURNING_LEFT → POST_TURN_BURST → DRIVE_FORWARD_LONG → IDLE`, 10 Hz.
  **All transitions are wall-clock time-based.** Sends `DutyM1M2(addr, right, left)` to RoboClaws
  `0x80` and `0x81` directly over `/dev/ttyAMA0` @38400. Declared ToF/PID wall-following
  (`kp/ki/kd`, `_left_wall_distance()`) exists but is **never executed** (dead code). No obstacle
  avoidance. RoboClaw `SetTimeout(addr, 0.5)` is the only runaway watchdog.
- **`camera_node`** (exe `camera`) — Arducam USB → `/camera/image_raw` (unused) + MJPEG on `:8080`.
  No navigation processing.
- **`input`** (`publisher_member_function`, node `minimal_publisher`) — WASD teleop → `topic` (Num) at
  20 Hz; **no key reset (no dead-man)** — last key repeats forever.
- **`drive_control`** (`subscriber_member_function`, node `minimal_subscriber`) — Num → `DutyM1M2` at a
  fixed 0.7 throttle to `0x80`/`0x81`; **no stop-on-shutdown**. NB: sends `DutyM1M2(addr, left, right)`
  — opposite arg order to `nav_state_machine`'s `(addr, right, left)`; reconcile before trusting either.

### Vision (`agrobot_detect`)
- **`vision_module.py`** — pure library (no rclpy). HSV `inRange` + contour pipeline, object tracking/
  counting at an entry line, `on_count_callback` is the only structured output. Computes yaw/pitch/
  distance but **never publishes them**, and the geometry is buggy (see §7).
- **`vision_application_publisher`** (exe; **node name is `lcd_display_publisher`** — collision) —
  runnable near-duplicate of the library; publishes `lcd_display/{text,color}`; `ENTRY_ZONE_WIDTH=0.35`.
- **`vision_lcd_publisher`** (node `vision_lcd_publisher`, **not in `setup.py`**) — clean producer
  wrapping `vision_module`; `ENTRY_ZONE_WIDTH=0.25`.
- **`beam_break_node`** (exe `beam_break_node`) — IR beam on BCM **GPIO27** → `/beam_break/blocked`,
  20 Hz, 3-sample debounce, active-low.

### Actuation (`agrobot_actuation`)
- **`remove.py`** (exe `remove`, node `remove_servo_server`) — serves `remove_servo`; AngularServo on
  **GPIO18**, sweep 0°→45° with 1 s holds (~2 s blocking in callback), `detach()` after. Thread-lock
  guards against concurrent motion (returns `BUSY`).
- **`beam_break_remove`** (exe `beam_break_remove`) — subscribes `/beam_break/blocked`, edge-triggered
  self-arming (clear→broken fires once), async-calls `/remove_servo`.
- **`seed.py`** (exe `seed`, node `seed_servo_server`) — serves `seed_servo`; AngularServo on
  **GPIO12**, sweep 20°→180° with 0.5 s holds (~1 s blocking).
- **`button_client`** (exe `button_client`) — physical buttons: **GPIO4 = remove**, **GPIO27 = seed**
  (⚠ conflicts with the beam sensor); calls both servos.

### Operator HMI (`agrobot_detect`)
- **`lcd_display_subscriber`** (exe) — owns the Waveshare SPI LCD; renders text + full-screen color.
- **`lcd_display_publisher`** (exe) — fake 3 s demo cycler (not the real vision producer).

### Motor driver (`basicmicro_ros2`, vendored — reference only)
- **`basicmicro_node`** + `servo_position_service_node` — full ros2_control RoboClaw driver
  (`cmd_vel`, `odom`, homing, absolute servo positioning, diagnostics). **Not launched on this robot** —
  the custom nodes drive the RoboClaws directly through the `basicmicro` Python lib instead.

---

## 5. Hardware / GPIO map (BCM numbering)

| Pin / bus | Device | Owner node | Backend |
|---|---|---|---|
| GPIO4 | remove push-button | `button_client` | gpiozero Button |
| GPIO6 / GPIO25 / GPIO26 | LCD RST / DC / Backlight | `lcd_display_subscriber` | RPi.GPIO (rpi-lgpio) |
| SPI0.0 (CE0/MOSI/SCLK) @4 MHz | Waveshare 2" ST7789 LCD | `lcd_display_subscriber` | spidev |
| GPIO12 | seed servo | `seed.py` | gpiozero + lgpio |
| GPIO18 | remove servo | `remove.py` | gpiozero + lgpio |
| **GPIO27** | **IR beam sensor** *and* **seed button** | `beam_break_node` **and** `button_client` | **⚠ CONFLICT** |
| I2C bus1 @0x70 → 8× 0x41 | TCA9548A mux → TMF8801 ×8 | `tof_sensor_node` | smbus2 |
| UART `/dev/ttyAMA0` (→ ttyACM0/ttyUSB0) @38400 | 2× RoboClaw `0x80`/`0x81` | `nav_state_machine` **and** `drive_control` | basicmicro lib (**shared port**) |
| CSI / USB `/dev/video0` | Pi cam (vision) / Arducam (nav) | vision nodes / `camera_node` | picamera2 / OpenCV |
| TCP `:8080` | MJPEG debug stream | `camera_node` **and** vision streaming | http.server (**shared port**) |

**Motor convention:** in the live nav path, `M1 = right side, M2 = left side` (`DutyM1M2(addr, right, left)`).
`drive_control` uses `(addr, left, right)` and the vendored `hardware_interface` uses yet another mapping —
**do not mix**; pick one convention and make all three agree.

---

## 6. End-to-end data-flow traces

**Weed removal (hardware reflex, no camera):**
`object crosses IR beam (GPIO27)` → `beam_break_node` (debounce) → `/beam_break/blocked` (Bool) →
`beam_break_remove` (False→True edge while armed) → `call_async /remove_servo` → `remove.py` sweeps
servo GPIO18 0°→45°.

**Seeding (manual only):**
`operator presses seed button (GPIO27)` → `button_client.call_seed` → `seed_servo` → `seed.py` sweeps
servo GPIO12 20°→180°. *No vision/nav trigger exists.*

**Navigation (open-loop):**
`tof_sensor_node → tof_distance_0..7` (subscribed by nav but **ignored**). Start/stop:
`input → topic (Num) → nav_state_machine` (`w`=start, `q`=stop). FSM runs burst/turn/burst/drive on
timers → `DutyM1M2` to both RoboClaws. Parallel manual path: `input → topic → drive_control → DutyM1M2`.

**Vision → HMI (telemetry only):**
`camera → HSV/contour pipeline → count event → lcd_display/color + lcd_display/text →
lcd_display_subscriber → SPI LCD`. Also MJPEG debug at `http://<pi>:8080`.

---

## 7. Known issues & gotchas (developer checklist)

**Blocking / dangerous**
- **GPIO27 double-claim** — never run `beam_break_node` and `button_client` together (beam sensor pin
  27 vs seed button pin 27). Move one.
- **Blind navigation** — nav ignores ToF; 8 s forward with no obstacle avoidance / e-stop. Only
  protection is the RoboClaw 0.5 s serial-timeout watchdog.
- **No software stop** — `drive_control` doesn't zero duty on exit; `input` has no dead-man. Killing
  `drive_control` can leave the robot driving.
- **Dual drive path / port contention** — `drive_control` and `nav_state_machine` both consume `topic`
  **and** both open `/dev/ttyAMA0`: command + serial-port contention if run together.

**Won't-run / build**
- `vision_lcd_publisher` missing from `agrobot_detect/setup.py` console_scripts.
- Node-name collision: `vision_application_publisher`'s node is named `lcd_display_publisher`.
- Port `:8080` double-bind (`camera_node` vs vision streaming).
- `package.xml` errors that break rosdep: `agrobot_actuation` has `std_msg` (→`std_msgs`) and
  `python3-pgiozero` (→`python3-gpiozero`); `agrobot_nav` declares `tutorial_interfaces` but not
  `agrobot_interfaces`.
- Hardcoded venv path `~/ros2_ws/venv/lib/python3.12/site-packages` in nav/drive nodes.
- Vendored `basicmicro_driver` default port `/dev/ttyACM0` is wrong for this robot (`/dev/ttyAMA0`).

**Correctness / quality**
- Vision geometry: bbox coords are computed on the 0.3×-rescaled frame (≈192×144) but compared to
  full-resolution centers (`self.width/2`), so yaw/pitch/distance are off by roughly the rescale
  factor. Moot today because they're never published — fix before wiring vision → actuation.
- `beam_break_remove`: no `call_async` timeout (can wedge), `armed=True` at boot (fires if it starts
  with the beam already broken).
- ToF: no calibration, accepts confidence as low as 1/63, no range clamp/smoothing, bare `Float32`
  (no stamp/frame_id), ~13 s blocking init.
- Placeholder metadata throughout (`root@todo.todo`, "TODO: Package description", tutorial names
  `minimal_publisher`/`topic`/`Num`).
- Field note (`agrobot_nav/agrobot_nav/notes.txt`): "TOF sensors too low — dirt reads as wall, turns
  too early; also turns right too long." (Consistent with nav being open-loop and un-tuned.)

---

## 8. How to build & run

```bash
cd ~/ros2_ws
source /opt/ros/kilted/setup.bash
source venv/bin/activate                     # picamera2/opencv/basicmicro live here
colcon build --symlink-install               # or --packages-select <pkg>
source install/setup.bash

# --- Navigation (open-loop) ---
ros2 launch agrobot_nav agrobot_launch.py    # brings up tof_sensor + nav
ros2 run agrobot_nav input                   # teleop: w=start / q=stop (separate terminal)
#   manual drive instead of FSM: ros2 run agrobot_nav drive_control

# --- Actuation ---
ros2 run agrobot_actuation remove            # remove_servo server (GPIO18)
ros2 run agrobot_actuation seed              # seed_servo server (GPIO12)
ros2 run agrobot_actuation beam_break_remove # auto-fire remove on beam break
#   or manual buttons:  ros2 run agrobot_actuation button_client   (⚠ GPIO27 clash with beam node)

# --- Vision + HMI ---
ros2 run agrobot_detect beam_break_node             # IR beam → /beam_break/blocked
ros2 run agrobot_detect vision_application_publisher # camera → lcd_display/*  (pick ONE producer)
ros2 run agrobot_detect lcd_display_subscriber       # drives the SPI LCD

# Serial sanity checks (see ~/notes.txt): confirm Pi UART loopback, then RoboClaw comms.
ros2 topic list ; ros2 topic echo /beam_break/blocked ; ros2 service list
```

---

## 9. Where to start for common tasks

- **Close the mission loop (vision → actuation):** have a vision node publish a weed/seed target or a
  per-class trigger, then add a decision node mapping detections to `remove_servo`/`seed_servo` calls.
  Fix the §7 geometry scale bug first if you need real angles/distance.
- **Make navigation sense-driven:** wire `_left_wall_distance()`/PID into `NavStateMachine._control_loop`,
  add a front-sensor obstacle check, and add an e-stop; fix the ToF mounting height (field note §7).
- **Arbitrate driving:** pick one command owner between `drive_control` and `nav_state_machine`, one
  `/dev/ttyAMA0` owner, one motor arg-order convention, and add stop-on-shutdown + a teleop dead-man.
- **Fix packaging so it builds clean:** add the `vision_lcd_publisher` entry point, rename the colliding
  node, correct the `package.xml` deps, and give `camera_node`/vision-streaming distinct ports.
- **Actuation tuning:** servo angles/holds are hardcoded constants at the top of `remove.py`/`seed.py`.
