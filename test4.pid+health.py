from machine import Pin, I2C, ADC, PWM
import time
from ssd1306 import SSD1306_I2C
import dht

# LED Configuration
LED_PIN = 25  # Built-in LED for Raspberry Pi Pico
led = Pin(LED_PIN, Pin.OUT)

# LDR Configuration
LDR_PIN = 26  # ADC pin for LDR
ldr = ADC(LDR_PIN)
LDR_THRESHOLD = 30  # Simulated light level threshold at 30

# DHT Sensor Configuration
DHT_PIN = 0  # GPIO pin connected to DHT22
dht_sensor = dht.DHT22(Pin(DHT_PIN))

# OLED Display Configuration
WIDTH = 128  # Display width
HEIGHT = 64  # Display height
i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)  # I2C configuration
display = SSD1306_I2C(WIDTH, HEIGHT, i2c)  # Initialize OLED display

# Servo Motor Configuration
SERVO_PIN = 20  # GPIO pin connected to the servo's PWM signal
servo = PWM(Pin(SERVO_PIN))
servo.freq(50)  # Set PWM frequency to 50Hz (standard for servos)

# PIR Sensor Configuration
PIR_PIN = 2
pir = Pin(PIR_PIN, Pin.IN)
L_PIN = 4
l = Pin(L_PIN, Pin.OUT)

# PID Control Variables
Kp = 2.0  # Proportional gain
Ki = 0.1  # Integral gain
Kd = 0.5  # Derivative gain
integral = 0
last_error = 0
target_temperature = 25.0  # Target temperature in Celsius
servo_angle = 0  # Current servo angle

# Task Timestamps
last_pid_time = 0
last_display_time = 0
last_ldr_time = 0
last_pir_time = 0
last_health_check_time = 0  # Health monitoring task timestamp

def set_servo_angle(angle):
    """Moves the servo to the specified angle."""
    if 0 <= angle <= 360:
        duty = int(1000 + (angle / 360) * 8000)
        servo.duty_u16(duty)

def read_sensor_data():
    """Reads temperature and humidity from the DHT sensor."""
    try:
        dht_sensor.measure()
        return dht_sensor.temperature(), dht_sensor.humidity()
    except OSError as e:
        print(f"Failed to read from DHT sensor: {e}")
        return None, None

def read_ldr():
    """Reads the LDR value from the ADC pin."""
    return ldr.read_u16()

def pid_control(current_temp, target_temp):
    """PID control logic to adjust servo angle based on temperature."""
    global integral, last_error, servo_angle
    error = target_temp - current_temp
    proportional = Kp * error
    integral += error * 0.1
    derivative = (error - last_error) / 0.1
    pid_output = proportional + Ki * integral + Kd * derivative
    servo_angle = max(0, min(360, servo_angle + pid_output))
    set_servo_angle(servo_angle)
    last_error = error

def update_display(temp, hum, servo_angle, ldr_value, pir_value):
    """Updates the OLED display with sensor data."""
    display.fill(0)
    display.text("Sensor Readings:", 0, 0)
    if temp is not None and hum is not None:
        display.text(f"Temp: {temp:.2f}C", 0, 16)
        display.text(f"Humidity: {hum:.2f}%", 0, 32)
    else:
        display.text("Sensor Error!", 0, 16)
    display.text(f"LDR: {ldr_value}", 0, 56)
    display.text("Intruders!" if pir_value else "No intruders", 0, 48)
    display.show()

def monitor_health():
    """Monitors system health and logs unusual behavior."""
    print("Health Check: Running diagnostics...")
    # Check DHT sensor
    try:
        dht_sensor.measure()
    except OSError:
        print("Error: DHT sensor is not responding!")
    # Check LDR reading
    ldr_value = read_ldr()
    if ldr_value == 0:
        print("Error: LDR value is unusually low!")
    # Check PIR sensor
    pir_value = pir.value()
    if pir_value not in [0, 1]:
        print("Error: PIR sensor returned invalid value!")
    print("Health Check: All systems functional.")

while True:
    current_time = time.ticks_ms()

    # PID Control (Every 100ms)
    if time.ticks_diff(current_time, last_pid_time) >= 100:
        temperature, humidity = read_sensor_data()
        if temperature is not None:
            pid_control(temperature, target_temperature)
        last_pid_time = current_time

    # Update Display (Every 2000ms)
    if time.ticks_diff(current_time, last_display_time) >= 2000:
        ldr_value = read_ldr()
        update_display(temperature, humidity, servo_angle, ldr_value, pir.value())
        last_display_time = current_time

    # LDR Control (Every 1000ms)
    if time.ticks_diff(current_time, last_ldr_time) >= 1000:
        ldr_value = read_ldr()
        new = (381445 / ldr_value)
        led.value(1 if new < LDR_THRESHOLD else 0)
        last_ldr_time = current_time

    # PIR Motion Detection (Every 500ms)
    if time.ticks_diff(current_time, last_pir_time) >= 500:
        pir_value = pir.value()
        l.value(pir_value)  # Turn light on/off based on PIR sensor
        last_pir_time = current_time

    # Health Monitoring (Every 10 seconds)
    if time.ticks_diff(current_time, last_health_check_time) >= 10000:
        monitor_health()
        last_health_check_time = current_time
