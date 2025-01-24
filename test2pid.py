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
HEIGHT = 64   # Display height
i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)  # I2C configuration
display = SSD1306_I2C(WIDTH, HEIGHT, i2c)  # Initialize OLED display

# Servo Motor Configuration
SERVO_PIN = 20  # GPIO pin connected to the servo's PWM signal
servo = PWM(Pin(SERVO_PIN))
servo.freq(50)  # Set PWM frequency to 50Hz (standard for servos)

# PIR Sensor Configuration
PIR_PIN = 2
pir = Pin(PIR_PIN, Pin.IN)
L_PIN=4
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

def set_servo_angle(angle):
    """Moves the servo to the specified angle."""
    if 0 <= angle <= 360:
        # Convert angle (0-360) to duty cycle (1000-9000 for 1ms to 2ms pulse)
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

    # Calculate error
    error = target_temp - current_temp

    # Proportional term
    proportional = Kp * error

    # Integral term
    integral += error * 0.1  # Assume 100ms loop
    integral_term = Ki * integral

    # Derivative term
    derivative = (error - last_error) / 0.1  # Derivative over 100ms
    derivative_term = Kd * derivative

    # PID output
    pid_output = proportional + integral_term + derivative_term

    # Update servo angle (clamp to 0-360)
    servo_angle = max(0, min(360, servo_angle + pid_output))

    # Set servo position
    set_servo_angle(servo_angle)

    # Update last error
    last_error = error

def update_display(temp, hum, servo_angle, ldr_value, pir_value):
    """Updates the OLED display with temperature, humidity, and other sensor data."""
    display.fill(0)  # Clear the display
    display.text("Sensor Readings:", 0, 0)

    # Display temperature and humidity
    if temp is not None and hum is not None:
        display.text(f"Temp: {temp:.2f}C", 0, 16)
        display.text(f"Humidity: {hum:.2f}%", 0, 32)
    else:
        display.text("Sensor Error!", 0, 16)

    # Display servo angle
    #display.text(f"Servo Angle: {servo_angle:.1f}", 0, 48)

    # Display LDR value
    display.text(f"LDR: {ldr_value}", 0, 56)

    # Display PIR status
    if pir_value == 0:
        display.text("No intruders", 0, 48)
        
    else:
        display.text("Intruders!", 0, 48)
        

    # Render the updated display
    display.show()

while True:
    current_time = time.ticks_ms()

    # PID Control (Every 100ms)
    if time.ticks_diff(current_time, last_pid_time) >= 100:
        temperature, humidity = read_sensor_data()
        if temperature is not None:
            pid_control(temperature, target_temperature)
        last_pid_time = current_time

    # Update Display (Every 500ms)
    if time.ticks_diff(current_time, last_display_time) >= 500:
        ldr_value = read_ldr()
        update_display(temperature, humidity, servo_angle, ldr_value, pir.value())
        last_display_time = current_time

    # LDR Control (Every 300ms)
    if time.ticks_diff(current_time, last_ldr_time) >= 300:
        ldr_value = read_ldr()
        new = (381445 / ldr_value)
        if new < LDR_THRESHOLD:
            led.value(1)  # Turn LED ON
        else:
            led.value(0)  # Turn LED OFF
        last_ldr_time = current_time

    # PIR Motion Detection (Every 300ms)
    if time.ticks_diff(current_time, last_pir_time) >= 300:
        pir_value = pir.value()
        if pir_value == 0:  # No motion detected
            l.value(0)
            time.sleep(0.1)
        else:  # Motion detected
            l.value(1)

            time.sleep(0.1)
        last_pir_time = current_time
