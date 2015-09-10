import serial

ser = serial.Serial('/dev/cu.LightBlue-Bean')
print ser
print ser.portstr
while 1:

    ser.timeout = 1
    value = ser.readline(35).replace("\r\n", " ")
    
    print value
    # print type(value)
    # print "----"

ser.close()