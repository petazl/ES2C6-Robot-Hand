# ES2C6 robotic arm firmware

Open `main.ino` in Arduino IDE, select **Arduino Uno** and the board's port, and build or upload the sketch. Keep `Stabilization.h` beside `main.ino`. The sketch uses the Arduino `Servo` and `Wire` libraries.

## Operating the arm

Start with the joystick centered, the arm supported, and the ultrasonic sensor pointed at clear space. The arm starts in manual mode. The three SG90s start at 90° and move toward the positions set on the pots.

| Input | Manual control | Servo pin |
| --- | --- | --- |
| Joystick left/right | Turn the base | D4 |
| Joystick up/down | Rotate the wrist | D8 |
| POT1 | Set shoulder angle | D5 |
| POT2 | Set elbow angle | D6 |
| POT3 | Set vertical wrist angle | D7 |

The two FS90Rs turn while you hold the stick and stop when you center it. Watch for cable twist as those joints rotate. Each pot sets a target angle for its SG90, and the firmware limits how quickly the target is approached.

- **Gripper.** An object within about 15 cm starts closing the gripper. The motor stops when the force reading reaches the threshold or the close timer expires. Tap and release the joystick button to open the gripper in either mode. After opening, move the object away so the gripper can rearm.
- **Stabilization.** Once calibrated, hold the joystick button for about 0.8 seconds to capture the current claw orientation. The status LED lights up. Joystick left/right trims heading and up/down trims roll. The pots resume control when you hold the button again to return to manual mode.

## Changing settings

Edit these values near the top of `main.ino`, then rebuild and upload. Change one setting at a time during bench testing.

| What to adjust | Settings |
| --- | --- |
| A centered joystick still gives a command | `STICK_CENTER` (512) and `STICK_DEADBAND` (32) are raw analog counts. If the stick is centered and a servo still creeps, tune that FS90R's `BASE_STOP_US` or `WRIST_ROLL_STOP_US` around 1500 µs. |
| Joystick direction or speed | Flip `JOYSTICK_X_DIRECTION` or `JOYSTICK_Y_DIRECTION` between `1` and `-1`. Lower `MANUAL_SWING_US` for less maximum drive. |
| SG90 travel and response | The `SHOULDER_*`, `ELBOW_*`, and `WRIST_TILT_*` angle limits start at 20–150°, 30–160°, and 20–160°. Set each pair within the assembled joint's clearance. `POT_SLEW_RATE` controls how quickly the command approaches the pot angle. |
| Gripper response | `GRIPPER_SETTINGS` is `{force threshold, PWM, closing direction, close timeout, open duration}`. Defaults are `{200, 150, HIGH, 2000, 10000}`. Force is a raw ADC reading, PWM ranges from 0 to 255, and both times are in milliseconds. Check both times on the bench so the motor spends as little time as possible driving against an end stop. |
| Detection range or hold time | `NEAR_ECHO_US` is the ultrasonic round-trip time, roughly 58 µs per centimetre. Keep it below `ECHO_TIMEOUT_US`. `LONG_PRESS_MS` sets the button hold time in milliseconds. |

### First stabilization setup

1. Set `IMU_FORWARD`, `IMU_LEFT`, and `IMU_UP` to match the mounted IMU. Use `±1` for sensor X, `±2` for Y, and `±3` for Z. Pick one of each so forward × left points up.
2. Verify `JOINT_DIRECTION` in base, shoulder, elbow, vertical wrist, wrist rotation order. A positive base correction turns left, positive pitch tips the claw down, and positive roll lifts its left side.
3. Set `stabilizationCalibrated` to `true` and upload. Keep the claw still for about a second after the servos settle so the gyro can calibrate. Start with small movements and return to manual mode if a joint corrects the wrong way.

`TRIM_RATE` sets how fast the joystick shifts the held orientation. `MAX_CORRECTION_RATE`, `WRIST_PITCH_RATE`, and `STABILIZATION_SWING_US` limit the correction commands. The proportional, integral, and derivative gains are in `Pid::update()` in `Stabilization.h`. Tune those after verifying the axes, joint directions, and neutral pulses.

## How the code works

`readMux()` selects one of the three pots, two stick axes, or the button on the 74HC4051 and discards the first reading after a channel change. The main loop checks the button and force sensor, samples distance every 60 ms, and updates arm control every 20 ms. The gripper has its own state machine so it can finish opening or wait for the object to clear while the arm remains controllable.

The stabilization filter combines gyro readings with gravity from the accelerometer. PID compares the estimated orientation with the captured target. The base corrects heading, the rotation wrist corrects roll, and the vertical wrist corrects pitch first, with help from the elbow and shoulder when needed. The mode holds orientation only. Heading is relative and can drift, and correction has less authority with the claw pointed nearly straight up or down. If an IMU read fails, continuous rotation stops for that update and the arm returns to manual mode.
