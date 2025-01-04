import time
import board
import digitalio
import alarm
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3
from busio import I2C
import simpleio
from adafruit_hid.mouse import Mouse
from digitalio import DigitalInOut, Direction, Pull

# imports needed for bluetooth
import adafruit_ble
from adafruit_ble.advertising import Advertisement
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.standard.hid import HIDService
from adafruit_ble.services.standard.device_info import DeviceInfoService

# Configuration
config = {
    'name': 'Right Mouse Ring',
    'left_btn': board.D3,
    'right_btn': board.D5,
    'scrollup_btn': board.D2,
    'scrolldown_btn': board.D1,
    'debounce_sleep': 0.15,
    'mouse_movement': True
}

# Button setup with correct pins from config
left_button = digitalio.DigitalInOut(config['left_btn'])
left_button.direction = digitalio.Direction.INPUT
left_button.pull = digitalio.Pull.UP

right_button = digitalio.DigitalInOut(config['right_btn'])
right_button.direction = digitalio.Direction.INPUT
right_button.pull = digitalio.Pull.UP

scroll_up_button = digitalio.DigitalInOut(config['scrollup_btn'])
scroll_up_button.direction = digitalio.Direction.INPUT
scroll_up_button.pull = digitalio.Pull.UP

scroll_down_button = digitalio.DigitalInOut(config['scrolldown_btn'])
scroll_down_button.direction = digitalio.Direction.INPUT
scroll_down_button.pull = digitalio.Pull.UP

# Turn on IMU and wait 50 ms
imu_pwr = digitalio.DigitalInOut(board.IMU_PWR)
imu_pwr.direction = digitalio.Direction.OUTPUT
imu_pwr.value = True
time.sleep(0.05)

# Set up I2C bus and initialize IMU
i2c = I2C(board.IMU_SCL, board.IMU_SDA)
sensor = LSM6DS3(i2c)

# Mouse movement parameters
mouse_min = -9
mouse_max = 9
step = (mouse_max - mouse_min) / 20.0

def steps(axis):
    return round((axis - mouse_min) / step)

# Filter parameters
alpha = 0.5  # Complementary filter parameter
scale_factor = 5  # Scale factor for mouse movement

# Initialize variables
filtered_x, filtered_y = 0, 0
last_time = time.monotonic()
clock = 0

# Sleep parameters
last_movement_time = time.monotonic()
sleep_threshold = 300  # 5 minutes of inactivity

def enter_sleep_mode():
    print("Entering sleep mode")
    # Set up right button as wake source
    wake_pin = alarm.pin.PinAlarm(pin=config['right_btn'], value=False, pull=True)
    # Enter deep sleep until button press
    print("Press right button to wake up")
    alarm.exit_and_deep_sleep_until_alarms(wake_pin)

def calibrate_sensor():
    print("Calibration mode. Follow the instructions...")
    time.sleep(1)  # Stabilize
    
    samples = 10
    sum_x = 0
    sum_y = 0
    
    for _ in range(samples):
        accel_x, accel_y, _ = sensor.acceleration
        sum_x += accel_x
        sum_y += accel_y
        time.sleep(0.1)
    
    global filtered_x, filtered_y
    filtered_x = sum_x / samples
    filtered_y = sum_y / samples
    print(f"Calibration complete. Offsets: X={filtered_x:.2f}, Y={filtered_y:.2f}")

def significant_movement_detected(x, y):
    movement_threshold = 2
    normalized_x = steps(x)
    normalized_y = steps(y)
    return abs(normalized_x) > movement_threshold or abs(normalized_y) > movement_threshold

# Setup for HID and BLE
hid = HIDService()
device_info = DeviceInfoService(software_revision=adafruit_ble.__version__,
                              manufacturer="Adafruit Industries")
advertisement = ProvideServicesAdvertisement(hid)
advertisement.appearance = 961
scan_response = Advertisement()
scan_response.complete_name = config['name']
ble = adafruit_ble.BLERadio()
ble.name = config['name']

# Setup mouse
mouse = Mouse(hid.devices)

# Debounce time
DEBOUNCE_TIME = None
debounce_delay = config['debounce_sleep']

def get_delay_time(delay):
    return time.monotonic_ns() + (delay * 1000000000)

# Long press threshold changed to 2 seconds
LONG_PRESS_THRESHOLD = 2  # 2 seconds for sleep mode trigger

print("Starting BLE advertising...")
ble.start_advertising(advertisement, scan_response)

while True:
    if not ble.connected:
        print("Waiting for connection...")
        if not ble.advertising:
            ble.start_advertising(advertisement, scan_response)
        while not ble.connected:
            pass
        print("Connected!")

    while ble.connected:
        # Check for calibration (both buttons pressed)
        if not right_button.value and not left_button.value:
            print("Calibration triggered")
            calibrate_sensor()
            last_movement_time = time.monotonic()
            continue

        # Read sensor data
        accel_x, accel_y, accel_z = sensor.acceleration
        gyro_x, gyro_y, gyro_z = sensor.gyro

        # Apply complementary filter
        current_time = time.monotonic()
        dt = current_time - last_time
        last_time = current_time

        filtered_x = alpha * (filtered_x + gyro_x * dt) + (1 - alpha) * accel_x
        filtered_y = alpha * (filtered_y + gyro_y * dt) + (1 - alpha) * accel_y

        # Handle mouse movement
        vertical_mov = simpleio.map_range(steps(filtered_x), 1.0, 20.0, 15.0, -15.0)
        horizontal_mov = simpleio.map_range(steps(filtered_y), 20.0, 1.0, -15.0, 15.0)

        # Move mouse based on IMU
        if significant_movement_detected(filtered_x, filtered_y):
            mouse.move(x=int(horizontal_mov), y=int(vertical_mov))
            last_movement_time = time.monotonic()

        # Handle button inputs with debouncing
        if not left_button.value:  # Left click
            if DEBOUNCE_TIME is None or DEBOUNCE_TIME < time.monotonic_ns():
                DEBOUNCE_TIME = get_delay_time(debounce_delay)
                mouse.press(Mouse.LEFT_BUTTON)
                while not left_button.value:
                    pass  # Wait for button release
                mouse.release(Mouse.LEFT_BUTTON)
                print("Left click")
                last_movement_time = time.monotonic()

        elif not right_button.value:  # Right click or sleep mode
            if DEBOUNCE_TIME is None or DEBOUNCE_TIME < time.monotonic_ns():
                DEBOUNCE_TIME = get_delay_time(debounce_delay)
                press_start_time = time.monotonic()
                mouse.press(Mouse.RIGHT_BUTTON)
                
                # Wait for button release or long press threshold
                while not right_button.value:
                    current_time = time.monotonic()
                    if current_time - press_start_time >= LONG_PRESS_THRESHOLD:
                        # Long press detected - enter sleep mode
                        mouse.release(Mouse.RIGHT_BUTTON)  # Release button before sleep
                        print("Long press detected - entering sleep mode")
                        enter_sleep_mode()
                        # After waking up
                        print("Waking up from sleep mode")
                        last_movement_time = time.monotonic()  # Reset activity timer
                        break
                
                # If we get here, it was a short press
                mouse.release(Mouse.RIGHT_BUTTON)
                print("Right click")
                last_movement_time = time.monotonic()

        elif not scroll_up_button.value:  # Scroll up
            if DEBOUNCE_TIME is None or DEBOUNCE_TIME < time.monotonic_ns():
                DEBOUNCE_TIME = get_delay_time(debounce_delay)
                mouse.move(wheel=1)
                print("Scroll up")
                last_movement_time = time.monotonic()

        elif not scroll_down_button.value:  # Scroll down
            if DEBOUNCE_TIME is None or DEBOUNCE_TIME < time.monotonic_ns():
                DEBOUNCE_TIME = get_delay_time(debounce_delay)
                mouse.move(wheel=-1)
                print("Scroll down")
                last_movement_time = time.monotonic()

        # Reset debounce if all buttons are released
        if (left_button.value and right_button.value and 
            scroll_up_button.value and scroll_down_button.value):
            DEBOUNCE_TIME = None

        # Debug output
        if (clock + 2) < time.monotonic():
            print("Filtered x", steps(filtered_x))
            print("Filtered y", steps(filtered_y))
            clock = time.monotonic()

    print("Disconnected")
    ble.start_advertising(advertisement, scan_response)
