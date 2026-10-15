# ESP32 Wi-Fi Turntable Controller

A local, standalone controller for a belt-reduced NEMA 17 turntable used during 3D scanning. The ESP32 hosts its browser UI and versioned REST/WebSocket API; it does not use a cloud service and does not control a camera.

## Status and hardware assumptions

This is an initial, **uncommissioned** implementation. Firmware and filesystem builds and host-side tests are separate from hardware verification; no motor, driver, power supply, or safety circuit has been tested by this repository.

| Item | Current assumption / unresolved detail |
| --- | --- |
| Controller | ESP32 DevKit V1, Arduino framework |
| Driver | A4988-compatible STEP/DIR driver; exact module and sense resistor are unknown |
| Motor | NEMA 17 bipolar, assumed 200 full steps/revolution; exact winding/current rating must be identified |
| Microstepping | 1/16; set MS1, MS2, and MS3 high on a standard A4988 module |
| Transmission | GT2 pulleys, 2 motor revolutions per table revolution |
| Position feedback | None: no encoder, home switch, or end stop |
| Safety feedback | No driver-fault input or physical emergency-stop circuit |
| Provisional software limits | 30°/s max speed, 10°/s² default acceleration, 30°/s² max acceleration |

Do not select a supply voltage or driver current limit until the exact motor and driver are identified and their datasheets are checked. Software speed limits are not verified mechanical limits.

## Architecture

- `include/config.h`: central GPIO, active polarity, mechanics, and provisional limits.
- `include/motion_core.h`: host-testable angle/pulse conversion, absolute direction policy, scan target generation, and settings validation.
- `src/main.cpp`: non-blocking AccelStepper motion servicing, manual/jog/continuous operation, scan state machine, REST API, WebSocket snapshots, AP, LittleFS, and NVS settings.
- `data/`: responsive static browser interface served from LittleFS.
- `test/test_motion_core.cpp`: hardware-independent C++ tests.

Motion is serviced with `AccelStepper::run()` from the main loop. HTTP, WebSocket, filesystem, and JSON work uses ESP32Async's asynchronous server. This avoids intentional long blocking movement loops, but asynchronous callbacks and Wi-Fi activity can still affect pulse timing; scope/logic-analyzer tests under network load are required before relying on the controller. AccelStepper and the actual motor/driver may not meet every requested speed or acceleration.

## GPIO and wiring

The proposed outputs avoid input-only GPIOs and common ESP32 boot-strapping pins. Verify the exact board pin labels before wiring.

| ESP32 DevKit V1 | Driver / hardware | Notes |
| --- | --- | --- |
| GPIO 26 | STEP | Step input |
| GPIO 27 | DIR | Direction input; verify which sign physically rotates clockwise |
| GPIO 25 | ENABLE / EN | Firmware assumes active-low, the usual A4988 behavior. **Verify the installed module**; the hardware may invert or expose different polarity. |
| 3V3 | VDD / logic supply | Driver logic only, if supported by the selected module |
| GND | Logic GND and motor-supply negative | Common logic reference required |
| Motor supply positive | VMOT | Separate motor supply; choose voltage only after module and motor datasheets are known |
| Driver 1A/1B and 2A/2B | Motor coil pairs | Identify winding pairs with a meter; connect one pair to 1A/1B and the other to 2A/2B |
| MS1, MS2, MS3 | Logic HIGH | For a standard A4988, all three HIGH selects 1/16 microstepping; verify module truth table |
| RESET and SLEEP | Tie together and pull HIGH | Typical A4988 arrangement; check the exact board/module documentation |
| VMOT to motor-supply GND | Decoupling capacitor | Place a suitable low-ESR electrolytic close to the driver VMOT/GND pins. Follow the driver/module datasheet's capacitance and voltage rating requirements; do not omit it. |

The ESP32 3.3 V rail must **never** power the stepper motor. Use a suitable regulated logic source and motor supply, with shared ground and wiring sized for the measured current. Do not connect or disconnect a stepper motor while the driver is powered; inductive transients can destroy the driver. Set current limit only after identifying the actual module's sense-resistor value and using that module's datasheet formula. Do not assume a generic `VREF` formula applies to an unidentified clone.

The firmware preloads STEP/DIR and inactive ENABLE output levels before configuring outputs, and does not move at boot. The driver remains disabled until an accepted enable or movement command. On reboot the software position is always **unverified**. Setting software zero records the current commanded position only; it cannot establish the actual mechanical orientation.

### Safety limitation

There is no physical emergency-stop circuit, end-stop switch, encoder, or driver-fault feedback in this initial design. Browser Stop is an ordinary controlled software stop, **not** a safety-rated emergency stop. For a final machine, add a reachable, appropriately rated physical means to remove motor power and/or inhibit driver enable, independently of Wi-Fi and firmware, following the machine's risk assessment. Disabling the driver removes holding torque; the table may move under external force.

## Motion and position model

Assumptions give `200 × 16 = 3,200` microsteps per motor revolution and `3,200 × 2 = 6,400` commanded pulses per table revolution. Thus:

- 17.777777… command pulses per degree.
- 0.05625° nominal command increment per pulse.
- 90° = 1,600 pulses; 360° = 6,400 pulses.
- At 30°/s, nominal STEP rate is 533.3 pulses/s and ideal constant-speed revolution time is 12 s, excluding acceleration/deceleration.

Absolute targets are rounded once from the requested unwrapped angle. Relative moves retain a fractional pulse accumulator so a series of small relative commands does not repeatedly discard fractions. Scan targets are computed from the original scan start and view index, not by repeated rounded increments. The displayed angle is normalized to `[0°, 360°)`; the unwrapped command estimate is also available in status.

Absolute movement has an explicit `directionPolicy`:

- `shortest` (default): choose the shortest signed path; an exactly half-turn tie moves in the positive/clockwise direction.
- `clockwise`: choose the positive/clockwise modular direction.
- `counterclockwise`: choose the negative/counterclockwise modular direction.

Direction is a software convention; verify the physical rotation direction during commissioning. Position values are estimates from commanded pulses, not encoder measurements or guaranteed table angles. Microstepping does not guarantee equivalent physical accuracy and missed steps cannot be detected without feedback. Disabling the driver invalidates position confidence. Set Zero while stopped to establish a new software reference.

## Scan behavior

The initial position is view 1. For `N` views over total angle `A`, target `i` is `start + i × A/N` for `i = 0 … N-1`; the endpoint is excluded. A 360°/36-view scan therefore targets 0°, 10°, …, 350°, never an extra 360° view. Increment mode targets the start and then each `k × increment` that is **strictly less** than the total angle; the endpoint is excluded to avoid a duplicate orientation. Its view count is `ceil(total/increment)`.

After movement and settling, a view becomes Ready. Default mode waits for `POST /api/v1/scan/acknowledge-view`; optional auto-advance advances after settling but does not claim a camera capture. Pause retains the scan and view; if stopped before reaching a target, Resume finishes the same target. A Ready view stays Ready across pause/resume. Cancel clears the scan. Stop decelerates; during a scan it pauses and retains the scan. Motor Disable is separate and is rejected while a scan remains active. Return-to-start is attempted only after a completed scan while position confidence remains valid; otherwise it is skipped with a status reason. No scan or physical position is resumed/restored after reboot.

A browser disconnect does not cancel an accepted scan. Jog instead uses a renewable lease (default 1,200 ms); disconnect/release stops the jog with controlled deceleration. Continuous rotation is a manual operation using replenished finite target segments and continues until Stop.

## Build and upload

Install PlatformIO Core and USB drivers for the board. The project pins the ESP32 platform and constrains library versions in `platformio.ini`.

```sh
pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor -b 115200
pio run -e esp32dev -t buildfs
pio run -e esp32dev -t uploadfs
```

The default ESP32 4 MB partition layout provides NVS and a filesystem partition used by LittleFS. `uploadfs` writes the filesystem image; it is not intended to erase NVS, but verify the selected board/partition and uploader behavior before the first deployment. `upload` normally updates the app partition without clearing NVS. Do not use an erase-flash command unless you intend to reset saved settings. On a filesystem mount failure the controller still starts, but there will be no web UI until a valid LittleFS image is uploaded.

The initial AP defaults are SSID `Turntable-Setup`, password `turntable`, URL `http://192.168.4.1/`. These are public factory defaults, not private credentials; change them in Settings for deployment. SSID/password and motion preferences persist in ESP32 Preferences/NVS. The Wi-Fi radio restarts with new AP credentials only after a reboot. Position, active motion, and scan progress are never persisted.

## Browser controls

Connect a phone or computer to the controller access point and open `http://192.168.4.1/`. The interface reconnects to the WebSocket and fetches a fresh status snapshot without replaying commands. Use Set Zero to establish the software coordinate before angle-based motion. Hold a jog button to renew its lease; releasing it requests deceleration. The custom and discrete angle controls are relative moves. Absolute moves offer all direction policies. Scan defaults are 360° and 36 views, with acknowledgement required at each Ready view.

## Versioned HTTP API

All endpoints use `/api/v1/`. JSON responses have `ok: true` on success or an `error` object containing `code` and `message` on error. Invalid/non-finite/out-of-range values and conflicting states are rejected with 4xx status. State-changing POST commands require a unique `operationId` (1–64 characters); recently processed IDs are cached so a retry is not executed twice. Use a new ID for a new intent. `PUT /settings` is idempotent and does not require an operation ID.

| Method and path | Purpose |
| --- | --- |
| `GET /api/v1/status` | Current estimated position, confidence, motor/motion, scan state |
| `POST /api/v1/move` | Relative or absolute finite move |
| `POST /api/v1/jog/start` | Start/renew leased jog |
| `POST /api/v1/jog/stop` | Controlled jog deceleration |
| `POST /api/v1/continuous/start` | Start continuous manual rotation |
| `POST /api/v1/stop` | Controlled stop; active scan becomes paused |
| `POST /api/v1/scan/start` | Begin an acknowledged or auto-advance scan |
| `POST /api/v1/scan/pause` / `resume` / `cancel` | Retain, resume, or clear scan state |
| `POST /api/v1/scan/acknowledge-view` | Advance exactly one Ready view |
| `POST /api/v1/position/zero` / `go-zero` | Set software zero / return to it |
| `POST /api/v1/motor/enable` / `disable` | Driver control; disable waits for motion stop |
| `GET /api/v1/settings` / `PUT /api/v1/settings` | Read/update validated NVS configuration |
| `WS /api/v1/events` | Initial and periodic JSON state snapshots |

Example relative move:

```http
POST /api/v1/move
Content-Type: application/json

{"operationId":"move-001","mode":"relative","degrees":90,"speedDegreesPerSecond":10}
```

Example absolute move; `directionPolicy` is required only if the default shortest-path policy is not desired:

```json
{"operationId":"move-002","mode":"absolute","degrees":270,"directionPolicy":"clockwise"}
```

Example 36-view acknowledged scan:

```json
{"operationId":"scan-001","mode":"views","totalDegrees":360,"views":36,"speedDegreesPerSecond":10,"autoAdvance":false,"returnToStart":false}
```

Example status (angles and pulses are estimates):

```json
{
  "ok": true,
  "position": {
    "estimatedAngle": 90,
    "unwrappedDegrees": 90,
    "absolutePulses": 1600,
    "zeroPulse": 0,
    "confidence": "estimated",
    "verified": true
  },
  "motor": {"enabled": true},
  "motion": "idle",
  "scan": {"active": false, "state": "idle", "viewIndex": 0, "viewCount": 36, "ready": false}
}
```

The status endpoint excludes the AP password. Settings accept `ssid`, `password`, `maxSpeedDegreesPerSecond`, `accelerationDegreesPerSecondSquared`, `settleMs`, and `jogLeaseMs`; omitted values remain unchanged. The server bounds speed to 0.5–30°/s, acceleration to 0.5–30°/s², settling to 0–60,000 ms, and jog lease to 300–5,000 ms. Firmware values must still be experimentally verified on the specific machine.

## Tests and commissioning

Host-side conversion and scan tests can run without ESP32 hardware:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude test/test_motion_core.cpp -o /tmp/test_motion_core
/tmp/test_motion_core
```

PlatformIO test/build commands:

```sh
pio test -e native
pio run -e esp32dev
pio run -e esp32dev -t buildfs
```

Commission cautiously:

1. Identify motor coil pairs with a multimeter and verify the exact motor, driver, logic voltage, motor supply, and sense resistor documentation.
2. With all power off, wire logic, common ground, decoupling, motor coils, and microstep pins. Never hot-plug motor wires.
3. Mechanically unload or safely secure the turntable. Ensure people, cables, and pinch points are clear; use an independent power isolation method.
4. Power logic with the driver disabled; confirm firmware starts without movement and that the AP/UI works.
5. Set the driver current limit from the actual module/motor specifications, beginning conservatively. Do not use an assumed supply voltage or generic resistor formula.
6. Enable the motor, verify direction with a very small low-speed jog, then verify software zero and 90°/360° pulse calculations.
7. Increase speed and acceleration gradually while checking missed steps, temperature, vibration, belt tension, driver temperature, and timing under web traffic. Stop if any behavior is unexpected.
8. Verify jog release and lease timeout, controlled stop, scan pause/resume/cancel/acknowledgement, browser disconnect behavior, restart behavior, and filesystem/NVS behavior.

Automated tests do not test the driver, actual angle, missed-step behavior, thermal performance, electrical noise, network-load timing, or physical safety. Those remain hardware commissioning tasks.
