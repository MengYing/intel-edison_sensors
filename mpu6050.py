import mraa
import time


def readOffset(x, num):
    temp = x.readReg(num) << 8 | x.readReg(num+1)
    if temp >= 0x8000:
        return -((65535 - temp) + 1)
    else:
        return temp


def checkFlag(args):
    global flag
    if flag is True:
        flag = False
    else:
        flag = True

global flag
flag = False
mpu6050_address = 0x68
pwr_mgmt_1 = 0x6b

mpu6050 = mraa.I2c(0)
mpu6050.address(mpu6050_address)
mpu6050.writeReg(pwr_mgmt_1, 0)
button = mraa.Gpio(2)
button.dir(mraa.DIR_IN)
button.isr(mraa.EDGE_RISING, checkFlag, checkFlag)
while True:
    if flag is True:
        accX = readOffset(mpu6050, 0x3b)
        accY = readOffset(mpu6050, 0x3d)
        accZ = readOffset(mpu6050, 0x3f)
        temp = readOffset(mpu6050, 0x41)/340.+36.53
        gyrX = readOffset(mpu6050, 0x43)
        gyrY = readOffset(mpu6050, 0x45)
        gyrZ = readOffset(mpu6050, 0x47)
        print accX, accY, accZ, temp, gyrX, gyrY, gyrZ
        time.sleep(0.5)
