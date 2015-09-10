import mraa  # calls the MRAA library
import time  # calls the time library

# Setup of variable and conditions
DecimalValue = 0.0  # Initialization of value converted
fsrReading = 0.0      # Initialization of value to be read
sensor = mraa.Aio(0)     # Use pin A0 in the Arduino Expansion Board


def valueMapping(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)
    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


def calResistance(fsrVoltage):
    #  Calculate sensor's resistance.
    #  Rsensor = (Rfixed*(Vtotal-Vout))/Vout (mohms)
    resistance = (10000000*(5000-fsrVoltage))/fsrVoltage
    return resistance


def calConductance(fsrResistance):
    # Caculate sensor's conductance.
    # C = 1/R(ohms)=1000000/R(mohms)
    conductance = 1000000/fsrResistance
    return conductance


def calLbs(fsrConductance):
    # Caculate lb force.
    # F = (C-0.0012)/0.00014
    forcelb = (fsrConductance-0.0012)/0.00014
    return forcelb


def calNewtons(fsrForcelb):
    # Caculate Newtons force.
    # N = F * 4.4;
    forceN = fsrForcelb * 4.4
    return forceN


try:
    while True:
        fsrReading = sensor.read()
        # Translate analog reading to Voltage(mV).
        fsrVoltage = valueMapping(fsrReading, 0.0, 1023.0, 0.0, 5000.0)

        if(fsrVoltage < 0.0001):
            print "no fsrVoltage\n==============="
            continue

        # caculate different values
        fsrResistance = calResistance(fsrVoltage)
        fsrConductance = calConductance(fsrResistance)
        fsrForcelb = calLbs(fsrConductance)

        if(fsrForcelb < 0.0001):
            print "no fsrForcelb\n==============="
            continue
        fsrForceN = calNewtons(fsrForcelb)

        print "fsrForcelb = ", fsrForcelb, "\t fsrForceN = ", fsrForceN,\
            "\n==============="

        time.sleep(1)   # Wait a second
except KeyboardInterrupt:
        print " "
