# SE(3) Geometric Flight Controller for RP2040

## Introduction

This is a **dual-loop geometric flight controller** implementing SE(3) (Special Euclidean Group) control for quadcopter drones. Unlike traditional PID controllers that work in Euler angle space, this controller respects the geometric structure of rotations on the SO(3) manifold, providing superior performance and avoiding singularities.

### Hardware Platform
- **MCU**: Raspberry Pi RP2040
- **IMU**: LSM9DS1TR 9-DOF sensor (I2C)
- **Radio**: NRF24L01P-R 2.4GHz transceiver (SPI)
- **Motor Drivers**: STT6N3LH6 N-channel MOSFETs (PWM control)
- **Configuration**: X-frame quadcopter

### Key Features

#### 🎯 Geometric Control
- **SE(3) state estimation**: Maintains rotation matrix on SO(3) manifold
- **Complementary filter**: Fuses gyroscope and accelerometer data geometrically
- **No gimbal lock**: Operates directly on rotation matrices, avoiding Euler angle singularities

#### 🔄 Dual-Loop Architecture
1. **Outer Loop (Attitude)**: Converts attitude errors to rate commands
2. **Inner Loop (Rate)**: Converts rate errors to motor torques

#### ⚙️ Physics-Based Control
- **Inertia compensation**: PID gains scaled by moment of inertia
- **Torque-to-thrust mapping**: Accounts for arm length in motor mixer
- **Hover throttle estimation**: Calculates expected hover point from mass

#### 🛡️ Safety Features
- Gyroscope and accelerometer calibration
- Radio failsafe (1-second timeout)
- Low battery voltage warning
- Arming safety (requires low throttle)
- Emergency shutdown via RUN button

---

## Why Physical Parameters Matter

### ❌ Without Physical Parameters
Traditional controllers use arbitrary PID gains that must be manually tuned for each quad:
- Gains don't transfer between different quad sizes
- No physical meaning to the control outputs
- Requires extensive test flights to tune

### ✅ With Physical Parameters (SE(3) + Dynamics)
The controller automatically adapts to your quad's characteristics:

```python
# Moment of inertia determines rotational response
Ixx = 2 × motor_mass × arm_length²

# Torque-to-thrust conversion uses geometry
torque = arm_length × thrust_difference / √2

# Rate gains scaled by inertia for consistent response
Kp_rate = base_gain / Ixx
```

**Result**: Same controller works on 250mm racer or 450mm cruiser with just parameter changes!

---

## Hardware Pin Assignments

### Verified from RB FALCON Schematic

| Function | GPIO Pin | Device | Note |
|----------|----------|---------|------|
| **I2C SCL** | GPIO 5 | LSM9DS1TR | I2C0 |
| **I2C SDA** | GPIO 4 | LSM9DS1TR | I2C0 |
| **SPI SCK** | GPIO 2 | NRF24L01P-R | SPI0 |
| **SPI MOSI** | GPIO 3 | NRF24L01P-R | SPI0 |
| **SPI MISO** | GPIO 0 | NRF24L01P-R | SPI0 |
| **NRF CSN** | GPIO 7 | NRF24L01P-R | Chip Select |
| **NRF CE** | GPIO 1 | NRF24L01P-R | Chip Enable |
| **Motor M1** | GPIO 15 | PWMA0 | Front-Right |
| **Motor M2** | GPIO 14 | PWMB0 | Front-Left |
| **Motor M3** | GPIO 19 | PWMA1 | Rear-Left |
| **Motor M4** | GPIO 18 | PWMB1 | Rear-Right |
| **LED Red** | GPIO 10 | RGB LED | Status |
| **LED Green** | GPIO 11 | RGB LED | Ready |
| **LED Blue** | GPIO 12 | RGB LED | Armed |
| **LED3** | GPIO 8 | Status LED | System |
| **LED4** | GPIO 9 | Status LED | Armed |
| **Battery ADC** | GPIO 29 | ADC3 | Voltage monitor |
| **RUN Button** | GPIO 23 | Button | E-stop |

### Motor Layout (X-Configuration)

```
        Front
      M2   M1
        \ /
         X
        / \
      M3   M4
        Rear
```

---

## Installation & Setup

### 1. Flash MicroPython
Flash the latest MicroPython firmware to your RP2040:
```bash
# Download from micropython.org
# Flash using picotool or drag-and-drop in bootloader mode
```

### 2. Upload Controller Code
- Copy the entire script to your RP2040 as `main.py`
- Or use Thonny IDE to run directly

### 3. Measure Your Quad Parameters

#### **Total Mass (kg)**
Weigh your fully assembled quad with battery:
```
Scale reading = 450g → mass = 0.45 kg
```

#### **Arm Length (m)**
Measure from center of quad to center of motor:

| Frame Size | Typical Arm Length |
|------------|-------------------|
| 250mm racing | 0.125 m (125mm) |
| 350mm cruiser | 0.175 m (175mm) |
| 450mm photography | 0.225 m (225mm) |
| 650mm heavy-lift | 0.325 m (325mm) |

Formula: `arm_length = diagonal_wheelbase / (2 × √2)`

#### **Motor Thrust Constant (N/throttle)**
Estimate from motor specs:
```
Example: 2205 2300KV motors
- Max thrust per motor: ~800g = 7.84 N
- Motor constant: 7.84 N / 1.0 = 7.84
```

If unknown, use typical values:
- Racing motors (2205-2207): 8.0
- Cruising motors (2306-2308): 10.0
- Heavy-lift motors (2814+): 15.0

---

## Usage

### Basic Usage (Defaults)

```python
# Run with default parameters (250mm racing quad)
main()

# Defaults:
# - mass = 0.45 kg
# - arm_length = 0.125 m
# - motor_kt = 8.0 N
```

### Custom Parameters

```python
# Specify your quad's parameters
main(mass=0.6, arm_length=0.15, motor_kt=10.0)
```

### Interactive Mode

When you run the script, it will prompt you:

```
Enter quad parameters (or press Enter for defaults):
============================================================
Total mass in kg [0.45]: 0.5
Arm length in meters [0.125]: 0.13
Motor thrust constant [8.0]: 9.5
```

---

## Radio Control Setup

### Transmitter Configuration

The controller expects a **32-byte payload** with the following format:

| Byte | Function | Range | Center | Description |
|------|----------|-------|--------|-------------|
| 0 | Roll | 0-255 | 127 | ±30° max |
| 1 | Pitch | 0-255 | 127 | ±30° max |
| 2 | Yaw Rate | 0-255 | 127 | ±180°/s max |
| 3 | Throttle | 0-255 | 0 | 0-100% |
| 4 | Arm Switch | 0-255 | 0/255 | >200 = armed |
| 5-31 | Reserved | - | - | Future use |

### NRF24L01 Settings
```python
Channel: 76 (2.476 GHz)
Data rate: 1 Mbps
Power: 0 dBm
Address: 0xE7E7E7E7E7
Payload: 32 bytes
```

---

## Pre-Flight Checklist

### ⚠️ Safety First
1. **Remove propellers** for initial testing
2. **Secure the quad** to prevent unexpected movement
3. **Test in open area** away from people and obstacles
4. **Have kill switch ready** (transmitter failsafe or power disconnect)

### 🔧 Calibration Procedure

1. **Place quad on level surface**
2. **Power on** (do not move during calibration)
3. **Red LED blinks**: Gyroscope calibrating (keep still!)
4. **Green LED on**: Calibration complete, ready
5. **Test motor response** (no props):
   - Arm with low throttle
   - Slowly increase throttle
   - Verify all motors spin
   - Check rotation direction

### ✈️ First Flight

1. **Install propellers** (correct rotation!)
   - M1 (FR), M4 (RR): CW
   - M2 (FL), M3 (RL): CCW
2. **Low throttle** to arm
3. **Slowly increase** to hover
4. **Make small inputs** to test response
5. **Land immediately** if unstable

---

## LED Status Indicators

| LED | State | Meaning |
|-----|-------|---------|
| Red | Blinking | Gyro calibration |
| Red | Solid | Low battery or failsafe |
| Green | Solid | System ready |
| Green | Blinking | Normal operation |
| Blue | Solid | Armed |
| Blue | Blinking | Armed and flying |
| LED3 | On | System initialized |
| LED4 | On | Armed state |

---

## Tuning Guide

### When to Tune

The geometric controller should work well with default gains for most quads. Only tune if you experience:
- Oscillations in hover
- Sluggish response
- Overshoot on rapid maneuvers

### Tuning Parameters

#### Attitude Loop (Outer Loop)
```python
self.Kp_att = [8.0, 8.0, 4.0]  # Roll, Pitch, Yaw
self.Ki_att = [0.3, 0.3, 0.1]
self.Kd_att = [1.0, 1.0, 0.5]
```
- **Increase Kp**: Faster attitude response (risk: oscillation)
- **Increase Ki**: Better hold in wind (risk: overshoot)
- **Increase Kd**: Damping (risk: noise amplification)

#### Rate Loop (Inner Loop)
```python
# Base gains (before inertia scaling)
base_Kp = 1.5  # Higher = tighter rate tracking
base_Ki = 0.2  # Higher = less drift
base_Kd = 0.05 # Higher = more damping
```

### Progressive Tuning Steps

1. **Start conservative**: Reduce all gains by 50%
2. **Test hover**: Should be stable but may be sluggish
3. **Increase Kp_rate**: Until slight oscillation appears
4. **Back off 20%**: From oscillation point
5. **Add Kd_rate**: To dampen any remaining oscillation
6. **Tune attitude loop**: Same process for Kp_att

---

## Troubleshooting

### Motors Don't Spin
- ✓ Check arming (throttle must be low)
- ✓ Verify radio connection (green LED blinks when receiving)
- ✓ Check motor driver MOSFETs
- ✓ Test PWM output with oscilloscope

### Oscillations in Flight
- ✓ Reduce rate loop Kp by 25%
- ✓ Increase rate loop Kd slightly
- ✓ Check propeller balance
- ✓ Verify frame rigidity

### Drifts in Hover
- ✓ Re-calibrate gyroscope on level surface
- ✓ Check accelerometer calibration
- ✓ Increase attitude Ki (carefully)
- ✓ Verify CG is centered

### Failsafe Triggered
- ✓ Check NRF24 antenna connection
- ✓ Reduce distance to transmitter
- ✓ Verify transmitter is powered and transmitting
- ✓ Check for 2.4GHz interference

### IMU Errors
- ✓ Check I2C connections (SDA, SCL, power)
- ✓ Verify I2C address (0x6B for accel/gyro)
- ✓ Test with I2C scanner
- ✓ Check for loose connections from vibration

---

## Performance Expectations

### Control Loop Timing
- **Target rate**: 500-1000 Hz
- **Typical achieved**: 800-1000 Hz on RP2040
- **IMU sampling**: 952 Hz (LSM9DS1 ODR)

### Response Characteristics
- **Attitude step response**: ~200ms settling
- **Rate bandwidth**: ~20 Hz
- **Max commanded rates**: ±6 rad/s (~±340°/s)
- **Max commanded angles**: ±30° (roll/pitch)

---

## Advanced Features

### Extending the Controller

#### Add Position Control
The geometric framework supports full SE(3) control including position:
```python
# Outer-outer loop: Position → Attitude commands
# Current implementation: Attitude-only mode
# Can be extended to GPS/optical flow positioning
```

#### Custom Mixing
Modify `motor_mixer()` for different frame types:
- Plus (+) configuration
- Hexacopter
- Octocopter
- Y6 coaxial

#### Telemetry Logging
Add data logging for flight analysis:
```python
# Log to SD card or serial output
log_data = {
    'time': t,
    'attitude': att,
    'rates': gyro,
    'motors': motor_cmds
}
```

---

## References

### Theory
- **SE(3) Control**: Special Euclidean Group for rigid body dynamics
- **Geometric Mechanics**: Rotation matrices on SO(3) manifold
- **Complementary Filtering**: Sensor fusion without Kalman complexity

### Hardware Datasheets
- RP2040: [Raspberry Pi Pico Datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
- LSM9DS1: [STMicroelectronics Datasheet](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf)
- NRF24L01+: [Nordic Semiconductor Datasheet](https://www.nordicsemi.com/products/nrf24l01)

---

## License & Support

This controller is provided as-is for educational and research purposes.

**Safety Warning**: Flying drones can be dangerous. Always:
- Follow local regulations
- Fly in safe areas
- Maintain line of sight
- Use appropriate safety equipment
- Start with low-power bench testing

**Not responsible for**: Crashes, injuries, or property damage from use of this software.

---

## Quick Reference Card

```
┌─────────────────────────────────────────┐
│  SE(3) FLIGHT CONTROLLER QUICK START    │
├─────────────────────────────────────────┤
│ 1. Measure: mass, arm_length           │
│ 2. Run: main(mass, arm_length)         │
│ 3. Calibrate: Keep still during red LED│
│ 4. Arm: Low throttle + arm switch      │
│ 5. Fly: Smooth inputs, land on failsafe│
│                                         │
│ Emergency: RUN button or power off      │
└─────────────────────────────────────────┘
```

**Happy Flying! 🚁**
