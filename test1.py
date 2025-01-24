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

# Shared variables for temperature and humidity
temperature = None
humidity = None
last_temperature = None  # Track last temperature
servo_swept = False  # Flag to indicate if the servo has swept

# OLED Display Configuration
WIDTH = 128  # Display width
HEIGHT = 64   # Display height
i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)  # I2C configuration
display = SSD1306_I2C(WIDTH, HEIGHT, i2c)  # Initialize OLED display

# PIR Sensor Configuration
PIR_PIN = 2
pir = Pin(PIR_PIN, Pin.IN)

# Servo motor configuration
SERVO_PIN = 20   # GPIO pin connected to the servo's PWM signal
servo = PWM(Pin(SERVO_PIN))
servo.freq(50)    # Set PWM frequency to 50Hz (standard for servos)

def set_servo_angle(angle):
    """Moves the servo to the specified angle."""
    if 0 <= angle <= 360:
        # Convert angle (0-360) to duty cycle (1000-9000 for 1ms to 2ms pulse)
        duty = int(1000 + (angle / 360) * 8000)
        servo.duty_u16(duty)
    

def read_ldr():
    """Reads the LDR value from the ADC pin."""
    return ldr.read_u16()

def read_sensor_data():
    """Reads temperature and humidity from the DHT sensor."""
    global temperature, humidity
    try:
        dht_sensor.measure()
        temperature = dht_sensor.temperature()
        humidity = dht_sensor.humidity()
    except OSError as e:
        print(f"Failed to read from DHT sensor: {e}")
        temperature = None
        humidity = None

def update_display(temp, hum, ldr_value, pir):
    """Updates the OLED display with temperature, Lux, and humidity data."""
    display.fill(0)  # Clear the display
    display.text("Sensor Readings:", 0, 0)

    # Display temperature and humidity
    if temp is not None and hum is not None:
        display.text(f"Temp: {temp:.2f}C", 0, 16)
        display.text(f"Humidity: {hum:.2f}%", 0, 32)
    else:
        display.text("Sensor Error!", 0, 16)

    # Display PIR status
    if pir.value() == 0:
        display.text("No intruders", 0, 48)
    else:
        display.text("Intruders detected", 0, 48)

    # Display LDR value
    display.text(f"LDR: {ldr_value}", 0, 56)

    # Render the updated display
    display.show()

while True:
    read_sensor_data()  # Read data from DHT22
    ldr_value = read_ldr()  # Read LDR value
    update_display(temperature, humidity, ldr_value, pir)  # Update the OLED display
    time.sleep(2)  # Wait before the next reading

    # LDR Control
    new = (381445 / ldr_value)
    if new < LDR_THRESHOLD:  # Low light condition
        led.value(1)  # Turn LED ON
    else:  # Bright light condition
        led.value(0)  # Turn LED OFF    

    # PIR Sensor (Motion)
    if pir.value() == 0:  # No motion detected
        time.sleep(0.5)  
    else:  # Motion detected
        time.sleep(0.5) 

    # Servo Control based on temperature
    if temperature != last_temperature:  # Check if temperature has changed
        last_temperature = temperature  # Update last temperature
        servo_swept = False  # Reset the sweep flag

    if not servo_swept:  # Only move servo if it hasn't swept
        if temperature <= 30:
            for angle in range(0, 359, 10):  # Sweep from 0° to 360°
                set_servo_angle(angle)
                time.sleep(0.1) 
            servo_swept = True  # Set flag to indicate sweeping is done
        else:
            for angle in range(360, -1, -10):  # Sweep from 360° to 0°
                set_servo_angle(angle)
                time.sleep(0.1)  
            servo_swept = True  # Set flag to indicate sweeping is done
