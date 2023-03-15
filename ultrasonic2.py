import RPi.GPIO as GPIO
import time

# Set the GPIO pins
GPIO.setmode(GPIO.BCM)
TRIG = 23
ECHO = 24
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Function to measure distance
def measure_distance():
    # Send a trigger pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for the echo response
    start_time = time.time()
    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    while GPIO.input(ECHO) == 1:
        end_time = time.time()

    # Calculate distance
    duration = end_time - start_time
    distance = duration * 17150
    distance = round(distance, 2)
    return distance

# Loop to continuously measure distance
while True:
    distance = measure_distance()
    print("Distance:", distance, "cm")
    time.sleep(1)