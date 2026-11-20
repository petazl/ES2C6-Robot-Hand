# ES2C6 revised arm firmware

This sketch follows `ES2C6 Robotic Hand Revised.kicad_sch` and runs on an Arduino Uno R3. Open `main.ino` in Arduino IDE, install the Arduino Servo library if needed, select Arduino Uno and the board's port, then verify and upload.

## Controls and wiring

| Control or device | Uno connection | Function |
| --- | --- | --- |
| Joystick X | A4 | Left/right turns the FS90R base on D4 |
| Joystick Y | A5 | Connected in the schematic and unused by the program |
| POT1 | A1 | Sets shoulder SG90 angle on D5 |
| POT2 | A2 | Sets elbow SG90 angle on D6 |
| POT3 | A3 | Sets vertical wrist SG90 angle on D7 |
| Joystick button | D11 | Press to release the gripper |
| Force sensor | A0 | Stops closing at the configured force reading |
| HC-SR04 | Trigger D8, echo D9 | Detects objects near the gripper |
| Gripper driver | Direction D2, PWM D3 | Closes and opens the gripper motor |
| Status LED | D10 | Lights while the firmware is running |

Center the joystick and support the arm before power-up. The base starts stopped. The three SG90s start at 90° and approach their pot positions at the configured slew rate. Each pot sets an angle directly. Holding the joystick left or right controls base speed and direction, and centering it commands the base to stop. Watch cable twist as the base rotates.

An object within about 15 cm starts closing the gripper. The motor stops when the force reading reaches the threshold or the close timeout expires. A button press starts release after 25 ms of debounce. Holding it produces one release event.

The gripper records time spent driving closed and reverses for that duration on release, capped at 10 seconds. Holding time is excluded. Partial and timed-out closing attempts accumulate until a release finishes. At startup, a release with no recorded closing time uses the full 10 seconds because the jaw position is unknown. After opening or a close timeout, three readings outside the detection zone rearm automatic closing. A missing echo also counts as clear.

## Changing parameters

Edit the constants near the top of `main.ino`, rebuild, and upload. Check direction and travel with the arm supported.

| Setting | Default and purpose |
| --- | --- |
| `STICK_CENTER`, `STICK_DEADBAND` | 512 and 32 raw ADC counts. Center readings from 480 through 544 command a stop. |
| `BASE_STOP_US` | 1500 µs. Adjust until the FS90R stays still with the joystick centered. |
| `MANUAL_SWING_US` | 300 µs. Reduce for a slower maximum base command. Output stays between `FS90R_MIN_US` and `FS90R_MAX_US`. |
| `JOYSTICK_X_DIRECTION`, `BASE_DIRECTION` | Use 1 or -1. Reversing either one reverses base movement. |
| `SHOULDER_DIRECTION`, `ELBOW_DIRECTION`, `WRIST_TILT_DIRECTION` | Use 1 or -1 to reverse a pot-controlled joint around its center. |
| `SHOULDER_MIN/MAX` | 20–150° command limits inherited from the previous sketch. |
| `ELBOW_MIN/MAX` | 30–160° command limits inherited from the previous sketch. |
| `WRIST_TILT_MIN/MAX` | 20–160° command limits inherited from the previous sketch. Check each joint's assembled clearance. |
| `SG90_MIN_US`, `SG90_MAX_US` | 1000 and 2000 µs for the nominal 0–180° command mapping. Check actual horn travel on the bench. |
| `POT_SLEW_RATE` | 120 commanded degrees per second. Sets how quickly the servo command approaches a changed pot target. |
| `NEAR_ECHO_US` | 875 µs, approximately 15 cm. Allow roughly 58 µs per centimetre. Keep it below `ECHO_TIMEOUT_US` (3000 µs). |
| `BUTTON_DEBOUNCE_MS` | 25 ms. Filters switch bounce. |

`GRIPPER_SETTINGS` contains `{force threshold, PWM, closing direction, close timeout, full opening time}`. The defaults are `{200, 150, HIGH, 2000, 10000}`. Force is a raw ADC reading from 0 to 1023, PWM is from 0 to 255, and both times are in milliseconds. Change `HIGH` to `LOW` if closing runs backwards. The two-second close cutoff needs bench calibration, especially if small objects take longer to reach. The 10-second full opening time is the measured full-stroke reference and the maximum release duration.

## Program flow

`setup()` stops the gripper and base, centers the SG90 commands, enables the button's internal pull-up, and lights the status LED. The main loop polls force and button inputs, samples distance every 60 ms, and updates the four joint commands every 20 ms.

`controlBase()` maps stick deflection to the FS90R pulse. `followPot()` reads each pot directly and moves its SG90 command toward the mapped angle. `updateGripper()` steps through READY, CLOSING, HOLDING, OPENING, and WAIT_CLEAR while joint control continues. The echo read can wait up to 3 ms for a response.

Use the external servo supply shown in the schematic with a common ground to the Uno. The base's neutral pulse and each SG90's pulse-to-angle mapping need checks on the actual servos. Release timing estimates jaw travel, so verify the object clears the jaws. The virtual tests check firmware behavior with simulated inputs, and physical performance needs a bench test.
