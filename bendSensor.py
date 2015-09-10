import mraa  # calls the MRAA library
import time  # calls the time library

# Setup of variable and conditions
bendReading = 0.0      # Initialization of value to be read
sensor = mraa.Aio(1)     # Use pin A1 in the Arduino Expansion Board


def valueMapping(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)
    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

try:
    while True:
        bendReading = sensor.read()
        # Translate analog reading to Voltage(mV).
        bendValue = valueMapping(bendReading, 320.0, 520.0, 0.0, 360.0)

        print "bendReading = ", bendReading, "\tbendValue = ", bendValue, "\n"
        time.sleep(1)   # Wait a second
except KeyboardInterrupt:
        print " "
