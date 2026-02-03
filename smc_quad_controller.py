"""
Sliding Mode Control (SMC) with Madgwick Sensor Fusion
Hardware: LSM9DS1TR (I2C), NRF24L01P-R (SPI), N-MOSFET PWM
Dual-rate control: Inner loop 500Hz, Outer loop 100Hz
"""

import machine
import time
import math
import sys
from machine import Pin, I2C, SPI, PWM, ADC

# ============================================================================
# Hardware Configuration
# ============================================================================

i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)

spi = SPI(0, baudrate=8000000, polarity=0, phase=0,
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))
nrf_csn = Pin(1, Pin.OUT, value=1)
nrf_ce = Pin(0, Pin.OUT, value=0)

motor_pins = [22, 18, 19, 21]
motors = [PWM(Pin(p)) for p in motor_pins]
for m in motors:
    m.freq(8000)
    m.duty_u16(0)

try:
    vbat_adc = ADC(Pin(29))
except:
    vbat_adc = None

led_r = Pin(17 Pin.OUT)
led_g = Pin(16, Pin.OUT)
led_b = Pin(25, Pin.OUT)
led1 = Pin(12, Pin.OUT)

run_btn = Pin(26, Pin.IN, Pin.PULL_UP)

# ============================================================================
# Physical Parameters
# ============================================================================

class QuadParameters:
    def __init__(self, mass, arm_length, motor_constant=1.0):
        self.mass = mass
        self.arm = arm_length
        self.motor_kt = motor_constant
        
        motor_mass = 0.025
        self.Ixx = 2 * motor_mass * arm_length**2
        self.Iyy = 2 * motor_mass * arm_length**2
        self.Izz = 4 * motor_mass * arm_length**2
        
        self.g = 9.81
        
        total_thrust = mass * self.g
        self.hover_throttle = (total_thrust / 4.0) / motor_constant if motor_constant > 0 else 0.5
        
        print(f"\n=== Quad Parameters ===")
        print(f"Mass: {mass:.3f} kg")
        print(f"Arm: {arm_length:.3f} m")
        print(f"Inertia: Ixx={self.Ixx:.6f}, Iyy={self.Iyy:.6f}, Izz={self.Izz:.6f}")
        print(f"Hover: {self.hover_throttle:.2%}")
        print("======================\n")

# ============================================================================
# LSM9DS1TR IMU Driver
# ============================================================================

class LSM9DS1:
    ACC_GYR_ADDR = 0x6B
    
    def __init__(self, i2c):
        self.i2c = i2c
        self.gyro_bias = [0, 0, 0]
        
        devices = i2c.scan()
        print(f"I2C devices: {[hex(d) for d in devices]}")
        self.init_sensor()
        
    def write_reg(self, addr, reg, data):
        self.i2c.writeto_mem(addr, reg, bytes([data]))
        
    def read_reg(self, addr, reg, n=1):
        return self.i2c.readfrom_mem(addr, reg, n)
        
    def init_sensor(self):
        try:
            who = self.read_reg(self.ACC_GYR_ADDR, 0x0F)[0]
            print(f"LSM9DS1 WHO_AM_I: 0x{who:02X}")
            
            self.write_reg(self.ACC_GYR_ADDR, 0x10, 0xC0)
            self.write_reg(self.ACC_GYR_ADDR, 0x20, 0xC8)
            
            time.sleep_ms(100)
            print("LSM9DS1 initialized")
        except Exception as e:
            print(f"IMU error: {e}")
            raise
        
    def read_gyro(self):
        data = self.read_reg(self.ACC_GYR_ADDR, 0x18, 6)
        x = self._to_signed(data[0] | (data[1] << 8))
        y = self._to_signed(data[2] | (data[3] << 8))
        z = self._to_signed(data[4] | (data[5] << 8))
        scale = (2000.0 / 32768.0) * (math.pi / 180.0)
        return [(x*scale - self.gyro_bias[0]), 
                (y*scale - self.gyro_bias[1]), 
                (z*scale - self.gyro_bias[2])]
        
    def read_accel(self):
        data = self.read_reg(self.ACC_GYR_ADDR, 0x28, 6)
        x = self._to_signed(data[0] | (data[1] << 8))
        y = self._to_signed(data[2] | (data[3] << 8))
        z = self._to_signed(data[4] | (data[5] << 8))
        scale = (16.0 * 9.81) / 32768.0
        return [x*scale, y*scale, z*scale]
        
    def calibrate_gyro(self, samples=2000):
        print("Gyro calibration...")
        led_r.value(1)
        sum_g = [0, 0, 0]
        
        for i in range(samples):
            data = self.read_reg(self.ACC_GYR_ADDR, 0x18, 6)
            x = self._to_signed(data[0] | (data[1] << 8))
            y = self._to_signed(data[2] | (data[3] << 8))
            z = self._to_signed(data[4] | (data[5] << 8))
            scale = (2000.0 / 32768.0) * (math.pi / 180.0)
            sum_g[0] += x*scale
            sum_g[1] += y*scale
            sum_g[2] += z*scale
            if i % 100 == 0:
                led_r.toggle()
            time.sleep_ms(1)
            
        self.gyro_bias = [s/samples for s in sum_g]
        led_r.value(0)
        print(f"Gyro bias: {[f'{b:.5f}' for b in self.gyro_bias]}")
        
    def _to_signed(self, val):
        return val if val < 32768 else val - 65536

# ============================================================================
# NRF24L01P-R Driver
# ============================================================================

class NRF24L01:
    def __init__(self, spi, csn, ce, payload_size=32):
        self.spi = spi
        self.csn = csn
        self.ce = ce
        self.payload_size = payload_size
        self.init_radio()
        
    def write_reg(self, reg, data):
        self.csn.value(0)
        if isinstance(data, int):
            self.spi.write(bytes([0x20 | reg, data]))
        else:
            self.spi.write(bytes([0x20 | reg]) + data)
        self.csn.value(1)
        
    def read_reg(self, reg, length=1):
        self.csn.value(0)
        self.spi.write(bytes([reg & 0x1F]))
        data = self.spi.read(length)
        self.csn.value(1)
        return data[0] if length == 1 else data
        
    def init_radio(self):
        self.ce.value(0)
        time.sleep_ms(5)
        
        self.write_reg(0x00, 0x0F)
        self.write_reg(0x01, 0x00)
        self.write_reg(0x02, 0x01)
        self.write_reg(0x03, 0x03)
        self.write_reg(0x05, 0x4C)
        self.write_reg(0x06, 0x07)
        self.write_reg(0x11, self.payload_size)
        
        rx_addr = b'\xE7\xE7\xE7\xE7\xE7'
        self.write_reg(0x0A, rx_addr)
        self.write_reg(0x07, 0x70)
        
        self.csn.value(0)
        self.spi.write(bytes([0xE1]))
        self.csn.value(1)
        self.csn.value(0)
        self.spi.write(bytes([0xE2]))
        self.csn.value(1)
        
        time.sleep_ms(5)
        self.ce.value(1)
        print("NRF24L01 initialized")
        
    def available(self):
        status = self.read_reg(0x07)
        return (status & 0x40) != 0
        
    def read_payload(self):
        self.csn.value(0)
        self.spi.write(bytes([0x61]))
        data = self.spi.read(self.payload_size)
        self.csn.value(1)
        self.write_reg(0x07, 0x40)
        return data

# ============================================================================
# Madgwick AHRS Filter
# ============================================================================

class MadgwickAHRS:
    def __init__(self, sample_freq=500.0, beta=0.1):
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.beta = beta
        self.sample_freq = sample_freq
        self.inv_sample_freq = 1.0 / sample_freq
        
        print(f"Madgwick: β={beta}, freq={sample_freq}Hz")
        
    def update(self, gx, gy, gz, ax, ay, az, dt=None):
        if dt is None:
            dt = self.inv_sample_freq
            
        q0, q1, q2, q3 = self.q
        
        a_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if a_norm < 0.01:
            return
        ax /= a_norm
        ay /= a_norm
        az /= a_norm
        
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _4q0 = 4.0 * q0
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _8q1 = 8.0 * q1
        _8q2 = 8.0 * q2
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        
        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
        s1 = _4q1*q3q3 - _2q3*ax + 4.0*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az
        s2 = 4.0*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az
        s3 = 4.0*q1q1*q3 - _2q1*ax + 4.0*q2q2*q3 - _2q2*ay
        
        norm = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if norm > 0:
            s0 /= norm
            s1 /= norm
            s2 /= norm
            s3 /= norm
        
        qDot1 = 0.5 * (-q1*gx - q2*gy - q3*gz)
        qDot2 = 0.5 * ( q0*gx + q2*gz - q3*gy)
        qDot3 = 0.5 * ( q0*gy - q1*gz + q3*gx)
        qDot4 = 0.5 * ( q0*gz + q1*gy - q2*gx)
        
        qDot1 -= self.beta * s0
        qDot2 -= self.beta * s1
        qDot3 -= self.beta * s2
        qDot4 -= self.beta * s3
        
        q0 += qDot1 * dt
        q1 += qDot2 * dt
        q2 += qDot3 * dt
        q3 += qDot4 * dt
        
        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q[0] = q0 / norm
        self.q[1] = q1 / norm
        self.q[2] = q2 / norm
        self.q[3] = q3 / norm
        
    def get_euler(self):
        q0, q1, q2, q3 = self.q
        
        sinr_cosp = 2.0 * (q0*q1 + q2*q3)
        cosr_cosp = 1.0 - 2.0 * (q1*q1 + q2*q2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (q0*q2 - q3*q1)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        
        siny_cosp = 2.0 * (q0*q3 + q1*q2)
        cosy_cosp = 1.0 - 2.0 * (q2*q2 + q3*q3)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]

# ============================================================================
# Sliding Mode Controller (Dual-rate)
# ============================================================================

class SlidingModeController:
    def __init__(self, quad_params):
        self.quad = quad_params
        
        # Outer loop (100Hz): Attitude SMC
        self.lambda_att = [5.0, 5.0, 3.0]
        self.k_int_att = [0.5, 0.5, 0.2]
        self.k_reach_att = [8.0, 8.0, 4.0]
        self.eta_att = [2.0, 2.0, 1.0]
        self.phi_att = [0.05, 0.05, 0.08]
        
        # Inner loop (500Hz): Rate SMC (Super-twisting)
        self.lambda_rate = [10.0, 10.0, 8.0]
        self.k1_rate = [2.5, 2.5, 2.0]
        self.k2_rate = [1.5, 1.5, 1.2]
        self.phi_rate = [0.1, 0.1, 0.15]
        
        self.att_int = [0, 0, 0]
        self.att_prev = [0, 0, 0]
        self.rate_int = [0, 0, 0]
        self.rate_prev = [0, 0, 0]
        
        self.rate_cmd = [0, 0, 0]
        
        print("\n=== SMC Controller ===")
        print(f"Outer: λ={self.lambda_att}, φ={self.phi_att}")
        print(f"Inner: k1={self.k1_rate}, k2={self.k2_rate}")
        print("======================\n")
        
    def reset(self):
        self.att_int = [0, 0, 0]
        self.rate_int = [0, 0, 0]
        self.att_prev = [0, 0, 0]
        self.rate_prev = [0, 0, 0]
        self.rate_cmd = [0, 0, 0]
        
    def _sign(self, x, boundary=0.0):
        if boundary > 0:
            return math.tanh(x / boundary)
        return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)
    
    def _sat(self, x, boundary):
        if abs(x) <= boundary:
            return x / boundary
        return self._sign(x, 0)
    
    def update_outer(self, sp, att, dt):
        """Outer loop: 100Hz"""
        for i in range(2):
            e = sp[i] - att[i]
            e_dot = (e - self.att_prev[i]) / dt if dt > 0 else 0
            
            if abs(e) < 0.5:
                self.att_int[i] += e * dt
                self.att_int[i] = max(min(self.att_int[i], 0.5), -0.5)
            
            s = e_dot + self.lambda_att[i] * e + self.k_int_att[i] * self.att_int[i]
            
            u_reach = -self.k_reach_att[i] * self._sat(s, self.phi_att[i])
            u_switch = -self.eta_att[i] * self._sign(s, self.phi_att[i])
            
            self.rate_cmd[i] = u_reach + u_switch
            self.rate_cmd[i] = max(min(self.rate_cmd[i], 6.0), -6.0)
            
            self.att_prev[i] = e
        
        self.rate_cmd[2] = sp[2]
        
    def update_inner(self, rate, dt):
        """Inner loop: 500Hz"""
        torque = [0, 0, 0]
        
        for i in range(3):
            e = self.rate_cmd[i] - rate[i]
            s = e
            
            self.rate_int[i] += self._sign(s, self.phi_rate[i]) * dt
            self.rate_int[i] = max(min(self.rate_int[i], 0.2), -0.2)
            
            if abs(s) > 1e-6:
                u_st1 = -self.k1_rate[i] * math.sqrt(abs(s)) * self._sign(s, self.phi_rate[i])
            else:
                u_st1 = 0.0
            u_st2 = -self.k2_rate[i] * self.rate_int[i]
            
            torque[i] = u_st1 + u_st2
            
            self.rate_prev[i] = e
        
        torque[0] *= self.quad.Ixx
        torque[1] *= self.quad.Iyy
        torque[2] *= self.quad.Izz
        
        return torque

# ============================================================================
# Motor Mixer
# ============================================================================

def motor_mixer(throttle, torque, quad_params):
    L = quad_params.arm
    roll, pitch, yaw = torque
    
    roll_t = roll / (L * 1.414) if L > 0 else 0
    pitch_t = pitch / (L * 1.414) if L > 0 else 0
    yaw_t = yaw / (L * 2.0) if L > 0 else 0
    
    m1 = throttle + roll_t - pitch_t - yaw_t
    m2 = throttle - roll_t - pitch_t + yaw_t
    m3 = throttle - roll_t + pitch_t - yaw_t
    m4 = throttle + roll_t + pitch_t + yaw_t
    
    return [max(0, min(1, m)) for m in [m1, m2, m3, m4]]

def set_motors(motor_list, values):
    for i, m in enumerate(motor_list):
        m.duty_u16(int(values[i] * 65535))

# ============================================================================
# Main Controller
# ============================================================================

def main(mass=0.45, arm_length=0.125, motor_kt=8.0):
    print("=" * 60)
    print(" SMC CONTROLLER + MADGWICK AHRS")
    print(" Inner: 500Hz | Outer: 100Hz")
    print("=" * 60)
    
    quad = QuadParameters(mass, arm_length, motor_kt)
    
    print("Init IMU...")
    imu = LSM9DS1(i2c)
    
    print("Init radio...")
    radio = NRF24L01(spi, nrf_csn, nrf_ce)
    
    print("Init Madgwick...")
    madgwick = MadgwickAHRS(sample_freq=500.0, beta=0.1)
    
    print("Init SMC...")
    controller = SlidingModeController(quad)
    
    for led in [led_r, led_g, led_b]:
        led.value(1)
        time.sleep_ms(150)
        led.value(0)
    
    print("\nCalibrating...")
    imu.calibrate_gyro(2000)
    
    led_g.value(1)
    led1.value(1)
    print("\n" + "=" * 60)
    print("READY - SMC dual-rate active")
    print("=" * 60 + "\n")
    
    armed = False
    failsafe = False
    last_rx = time.ticks_ms()
    setpoint = [0, 0, 0, 0]
    
    last_inner = time.ticks_us()
    last_outer = time.ticks_us()
    last_print = time.ticks_ms()
    
    inner_count = 0
    outer_count = 0
    
    try:
        while True:
            now = time.ticks_us()
            dt_inner = time.ticks_diff(now, last_inner) / 1e6
            
            # Inner: 500Hz
            if dt_inner >= 0.002:
                last_inner = now
                
                try:
                    gyro = imu.read_gyro()
                    accel = imu.read_accel()
                except:
                    continue
                
                madgwick.update(gyro[0], gyro[1], gyro[2],
                               accel[0], accel[1], accel[2], dt_inner)
                att = madgwick.get_euler()
                
                if armed and not failsafe and setpoint[3] > 0.05:
                    torque = controller.update_inner(gyro, dt_inner)
                    motor_cmds = motor_mixer(setpoint[3], torque, quad)
                    set_motors(motors, motor_cmds)
                else:
                    set_motors(motors, [0, 0, 0, 0])
                
                inner_count += 1
                
            # Outer: 100Hz
            dt_outer = time.ticks_diff(now, last_outer) / 1e6
            if dt_outer >= 0.010:
                last_outer = now
                controller.update_outer(setpoint, att, dt_outer)
                outer_count += 1
            
            # Radio
            if radio.available():
                try:
                    data = radio.read_payload()
                    
                    setpoint[0] = (data[0] - 127) * (30 * math.pi / 180) / 127
                    setpoint[1] = (data[1] - 127) * (30 * math.pi / 180) / 127
                    setpoint[2] = (data[2] - 127) * (180 * math.pi / 180) / 127
                    setpoint[3] = max(0, min(1, data[3] / 255.0))
                    
                    arm_cmd = data[4] > 200
                    
                    if arm_cmd and not armed and setpoint[3] < 0.05:
                        armed = True
                        controller.reset()
                        led_b.value(1)
                        led4.value(1)
                        print("*** ARMED (SMC) ***")
                    elif not arm_cmd and armed:
                        armed = False
                        led_b.value(0)
                        led4.value(0)
                        print("*** DISARMED ***")
                    
                    last_rx = time.ticks_ms()
                    failsafe = False
                except:
                    pass
            
            # Failsafe
            if time.ticks_diff(time.ticks_ms(), last_rx) > 1000:
                if not failsafe:
                    failsafe = True
                    armed = False
                    print("*** FAILSAFE ***")
                    led_r.value(1)
            else:
                led_r.value(0)
            
            # Status
            if time.ticks_diff(time.ticks_ms(), last_print) > 500:
                r, p, y = [a * 180 / math.pi for a in att]
                print(f"R:{r:6.1f}° P:{p:6.1f}° Y:{y:6.1f}° | "
                      f"In:{inner_count*2}Hz Out:{outer_count*2}Hz | "
                      f"T:{setpoint[3]*100:5.1f}% | {'ARM' if armed else 'DIS'}")
                
                inner_count = 0
                outer_count = 0
                last_print = time.ticks_ms()
                led_g.toggle()
            
            if not run_btn.value():
                break
                
    except KeyboardInterrupt:
        print("\nShutdown")
    finally:
        set_motors(motors, [0, 0, 0, 0])
        for led in [led_r, led_g, led_b, led1]:
            led.value(0)
        nrf_ce.value(0)
        print("Stopped")

if __name__ == "__main__":
    default_mass = 0.45
    default_arm = 0.125
    default_kt = 8.0
    
    mass = default_mass
    arm_length = default_arm
    motor_kt = default_kt
    
    print("\nEnter parameters (Enter for defaults):")
    try:
        inp = input(f"Mass kg [{default_mass}]: ").strip()
        if inp: mass = float(inp)
        inp = input(f"Arm m [{default_arm}]: ").strip()
        if inp: arm_length = float(inp)
        inp = input(f"Motor Kt [{default_kt}]: ").strip()
        if inp: motor_kt = float(inp)
    except:
        print("Using defaults")
    
    print(f"\nConfig: {mass}kg, {arm_length}m, {motor_kt}N")
    main(mass=mass, arm_length=arm_length, motor_kt=motor_kt)
