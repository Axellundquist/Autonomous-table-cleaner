# Autonomous-table-cleaner
Autonomous tabletop cleaning robot developed in an electronics project course. Features ultrasonic-based edge and obstacle detection, encoder-based motor feedback, and PI-controlled differential drive for surface coverage.

---

## Demo / Overview

- **Goal:** cover/“sweep” a tabletop area while staying on the table and avoiding obstacles.
- **Safety:** downward-facing sensors detect table edges; the robot backs off and re-orients.
- **Navigation:** simple state-machine behavior with repeated passes across the table.

> `![demo](images/demo.gif)`

---

## Hardware

**Controller**
- Arduino Mega 2560
- Arduino Motor Shield

**Actuation**
- 2× Dagu DG02SS DC gear motors (differential drive)

**Sensing**
- 2× downward ultrasonic sensors (left/right) for **edge detection**
- 1× forward ultrasonic sensor for **obstacle detection**
- 1× forward-left ultrasonic sensor for **side obstacle detection**
- Wheel encoders (interrupt-driven pulse counting)

---

## Wiring Notes (high-level)

Motor Shield connections:
- **M1 = Left motor**
- **M2 = Right motor**

Ultrasonic sensors:
- Forward sensor: `TRIG_PIN_forward_horizontal`, `ECHO_PIN_forward_horizontal`
- Forward-left sensor: `TRIG_PIN_forward_left`, `ECHO_PIN_forward_left`
- Bottom (edge) sensors: `TRIG_PIN_left/ECHO_PIN_left` and `TRIG_PIN_right/ECHO_PIN_right`

Encoders:
- Left encoder interrupt: `SENSOR_PIN_LEFT`
- Right encoder interrupt: `SENSOR_PIN_RIGHT`

Pin mappings are defined at the top of the `.ino` file.

---

## Control & Logic (what the code does)

### 1) Heading stabilization while driving
Wheel encoder pulses are used to estimate relative wheel motion. A PI-style correction adjusts PWM to keep the robot moving straight when driving forward.

### 2) Edge detection and recovery
Downward ultrasonic sensors measure distance to the table surface. When one sensor reads “no surface” (large distance), the robot:
1. Stops
2. Estimates approach angle
3. Backs off a short distance
4. Turns ~180° (with correction) to continue coverage without leaving the table

### 3) Obstacle detection + avoidance
When the forward sensor detects an obstacle, the robot:
- Rotates to scan obstacle geometry (left/right)
- Executes a multi-step avoidance maneuver using a simple state machine

---

## Repository Structure (suggested)

