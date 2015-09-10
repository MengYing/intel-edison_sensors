import mraa
import time


MPU9250_ADDRESS = 0x68
pwr_mgmt_1 = 0x6B
MPU9250_WHO_AM_I = 0x75
WHO_AM_I_AK8963 = 0x00  # should return 0x48
# acc anf gyro register maps
MPU9250_ACCEL_XOUT_H = 0x3B
MPU9250_ACCEL_XOUT_L = 0x3C
MPU9250_ACCEL_YOUT_H = 0x3D
MPU9250_ACCEL_YOUT_L = 0x3E
MPU9250_ACCEL_ZOUT_H = 0x3F
MPU9250_ACCEL_ZOUT_L = 0x40
MPU9250_TEMP_OUT_H = 0x41
MPU9250_TEMP_OUT_L = 0x42
MPU9250_GYRO_XOUT_H = 0x43
MPU9250_GYRO_XOUT_L = 0x44
MPU9250_GYRO_YOUT_H = 0x45
MPU9250_GYRO_YOUT_L = 0x46
MPU9250_GYRO_ZOUT_H = 0x47
MPU9250_GYRO_ZOUT_L = 0x48
MPU9250_MAG_CNTL = 0x0A
MPU9250_MAG_ASAX = 0x10
MPU9250_MAG_ASAY = 0x11
MPU9250_MAG_ASAZ = 0x12
MPU9250_MAG_ST1 = 0x02


# magnetometer register maps
MAG_ADDRESS = 0x0C
MPU9250_WHO_AM_I = 0x75
MPU9250_MAG_XOUT_L = 0x03
MPU9250_MAG_XOUT_H = 0x04
MPU9250_MAG_YOUT_L = 0x05
MPU9250_MAG_YOUT_H = 0x06
MPU9250_MAG_ZOUT_L = 0x07
MPU9250_MAG_ZOUT_H = 0x08


GYRO_FULL_SCALE_250_DPS = 0x00
GYRO_FULL_SCALE_500_DPS = 0x08
GYRO_FULL_SCALE_1000_DPS = 0x10
GYRO_FULL_SCALE_2000_DPS = 0x18
ACC_FULL_SCALE_2_G = 0x00
ACC_FULL_SCALE_4_G = 0x08
ACC_FULL_SCALE_8_G = 0x10
ACC_FULL_SCALE_16_G = 0x18

INT_PIN_CFG = 0x37
INT_ENABLE = 0x38


def hex2dec(s):
    """return the integer value of a hexadecimal string s"""
    return int(str(s), 16)

'''
def readOffset(x, Register):
    temp = x.readReg(Register) << 8 | x.readReg(Register+1)
    if temp >= 0x8000:
        # return 0
        return -((65535 - temp) + 1)
    else:
        return temp


def I2Cread(mpu, Register, Nbytes, Data):
    # This function read Nbytes bytes from I2C device at address Address.
    # Put read bytes starting at register Register in the Data array.
    # Set register address
    mpu.readBytesReg(Register, Data)
'''


def I2CwriteByte(mpu, Register, Data):
    """ all parameter is uint8_t """
    # Write a byte (Data) in device (Address) at register (Register)
    # Set register address
    mpu.writeReg(Register, Data)


def initMPU9250():
    # wake up device
    mpu9250.writeReg(pwr_mgmt_1, 0)
    # Clear sleep mode bit (6), enable all sensors
    time.sleep(0.01)
    # Wait for all registers to reset

    # get stable time source
    mpu9250.writeReg(pwr_mgmt_1, 1)
    # Auto select clock source to be PLL gyroscope reference if ready else
    time.sleep(0.01)

    # Set accelerometers low pass filter at 5Hz
    mpu9250.writeReg(29, 0x06)
    # Set gyroscope low pass filter at 5Hz
    mpu9250.writeReg(26, 0x06)
    # Configure gyroscope range
    mpu9250.writeReg(27, 0x18)
    # Configure accelerometers range
    mpu9250.writeReg(28, 0x18)

    # Configure Interrupts and Bypass Enable
    # Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    # until interrupt cleared,
    # clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    # can join the I2C bus and all can be controlled by the Arduino as master
    ##mpu9250.writeReg(INT_PIN_CFG, 0x22)
    ##time.sleep(0.01)
    # Enable data ready (bit 0) interrupt
    mpu9250.writeReg(INT_ENABLE, 0x01)

    time.sleep(0.01)


def initMagnetometers():
    # power down magnetometer
    magnetometers.writeReg(MPU9250_MAG_CNTL, 0x00)
    time.sleep(0.01)

    # Enter Fuse ROM access mode
    magnetometers.writeReg(MPU9250_MAG_CNTL, 0x0F)
    time.sleep(0.01)

    magCalx = (float)(magnetometers.readReg(0x10) - 128)/256. + 1.
    magCaly = (float)(magnetometers.readReg(0x11) - 128)/256. + 1.
    magCalz = (float)(magnetometers.readReg(0x12) - 128)/256. + 1.

    print "magCal:"
    print magCalx, magCaly, magCalz
    # power down magnetometer
    magnetometers.writeReg(MPU9250_MAG_CNTL, 0x00)
    time.sleep(0.01)

    ##mpu9250.writeReg(0x37, 0x02)
    ##time.sleep(0.01)
    # magnetometers.writeReg(0x0A, 0x16)


def readAccGyroData():
    # Create 16 bits values from 8 bits data
    # Accelerometer
    # compare to readWordReg() function
    ax_16 = -(mpu9250.readReg(0x3b) << 8 | mpu9250.readReg(0x3C))
    # print mpu9250.readReg(0x3b) << 8, mpu9250.readReg(0x3C), \
    #    mpu9250.readReg(0x3b)
    ay_16 = -(mpu9250.readReg(0x3D) << 8 | mpu9250.readReg(0x3E))
    az_16 = mpu9250.readReg(0x3F) << 8 | mpu9250.readReg(0x40)

    # Gyroscope
    gx_16 = -(mpu9250.readReg(0x43) << 8 | mpu9250.readReg(0x44))
    gy_16 = -(mpu9250.readReg(0x45) << 8 | mpu9250.readReg(0x46))
    gz_16 = mpu9250.readReg(0x47) << 8 | mpu9250.readReg(0x48)

    print "Acc : "
    print ax_16, ay_16, az_16
    print hex2dec(str(ax_16)), hex2dec(str(ay_16)), hex2dec(str(az_16))
    print "Gpro: "
    print gx_16, gy_16, gz_16
    print hex2dec(str(gx_16)), hex2dec(str(gy_16)), hex2dec(str(gz_16))


def readMagData():
    # wait for magnetometer data ready bit to be set
    verify = magnetometers.readReg(MPU9250_MAG_ST1)
    # while(not(verify & 0x01)):
    #    verify = magnetometers.readReg(MPU9250_MAG_ST1) & 0x01
    magx_16 = -(magnetometers.readReg(0x04) << 8 | magnetometers.readReg(0x03))
    magy_16 = -(magnetometers.readReg(0x06) << 8 | magnetometers.readReg(0x05))
    magz_16 = -(magnetometers.readReg(0x08) << 8 | magnetometers.readReg(0x07))

    print "Magn: "
    print magx_16, magy_16, magz_16
    print hex2dec(str(magx_16)), hex2dec(str(magy_16)), hex2dec(str(magz_16))


def initialize():
    
    mpu9250.writeReg(0x6B, 0x80)  # Reset Device __MPUREG_PWR_MGMT_1
    time.sleep(0.01)
    mpu9250.writeReg(0x6B, 0x01)  # Clock Source
    time.sleep(0.01)
    mpu9250.writeReg(0x6A, 0x20)  # I2C Master mode __MPUREG_USER_CTRL
    time.sleep(0.01)
    mpu9250.writeReg(0x25, 0x0C)  # Set the I2C slave addres of AK8963 and set for write.
    time.sleep(0.01)
    mpu9250.writeReg(0x26, 0x0B)  # I2C slave 0 register address from where to begin data transfer
    time.sleep(0.01)

    mpu9250.writeReg(0x63, 0x01)  # Reset AK8963 __MPUREG_I2C_SLV0_DO
    time.sleep(0.01)
    mpu9250.writeReg(0x27, 0x81)  # Enable I2C and set 1 byte __MPUREG_I2C_SLV0_CTRL
    time.sleep(0.01)
    mpu9250.writeReg(0x26, 0x0A)  # I2C slave 0 register address from where to begin data transfer
    time.sleep(0.01)
    mpu9250.writeReg(0x63, 0x12)  # Register value to continuous measurement in 16bit
    time.sleep(0.01)
    mpu9250.writeReg(0x27, 0x81)  # Enable I2C and set 1 byte __MPUREG_I2C_SLV0_CTRL
    time.sleep(0.01)
    ##
    #mpu9250.writeReg(0x37, 0x02)
    #time.sleep(0.01)
    #mpu9250.writeReg(0x37, 0x22)
    #time.sleep(0.01)


if __name__ == '__main__':

    mpu9250 = mraa.I2c(0)
    mpu9250.address(MPU9250_ADDRESS)
    # Read the MPU9250_WHO_AM_I register of the magnetometer, this is a good test of communication
    a = mpu9250.readWordReg(MPU9250_WHO_AM_I)  # Read MPU9250_WHO_AM_I register for mpu9250
    time.sleep(0.01)
    print "mpu9250 I AM " + hex(a) + " I should be " + hex(0x71)
    
    # initial i2c setting for mpu9250
    initMPU9250()
    initialize()
    # readAccGyroData()

    ########

    # mpu9250.writeReg(0x37, 0x02)
    magnetometers = mraa.I2c(1)
    magnetometers.address(0x0c)
    # mpu9250.writeReg(0x37, 0x02)
    time.sleep(0.01)

    t = magnetometers.readWordReg(WHO_AM_I_AK8963)  # Read MPU9250_WHO_AM_I register for mpu9250
    time.sleep(0.01)
    print "AK8963 I AM " + hex(t) + " I should be " + hex(0x48)

    # mpu9250.writeReg(0x37, 0x02)
    # mpu9250.writeReg(0x37, 0x22)
    # mpu9250.writeReg(0x38, 0x01)
    time.sleep(0.01)
    readMagData()
    #magnetometers.writeReg(0x38, 0x01)
    time.sleep(0.01)
    readMagData()
    # Enable data ready (bit 0) interrupt
    # magnetometers.writeReg(INT_ENABLE, 0x01)

    # time.sleep(0.1)

    # magnetometers.writeReg(0x0A, 0x01)

    print magnetometers.readByte(), magnetometers.readReg(WHO_AM_I_AK8963)
    # Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    d = magnetometers.readWordReg(0x00)  # Read WHO_AM_I register for AK8963
    print "AK8963 I AM " + hex(d) + " I should be " + hex(0x48)
    print d
    initMagnetometers()
    readAccGyroData()
    readMagData()
    # readMagData()

    '''
    initMagnetometers()
    readMagData()
    print "=========="
    readAccGyroData()
    readMagData()
    time.sleep(0.1)

    readAccGyroData()
    readMagData()
    time.sleep(0.1)
    '''
    # Set by pass mode for the magnetometers
    # I2CwriteByte(mpu9250, 0x37, 0x02)
    # mpu9250.writeWordReg(0x37, 0x02)

    # Request continuous magnetometer measurements in 16 bits
    # I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16)
    # MAG_ADDRESS.writeWordReg(0x0A, 0x01)

    while False:
        # accelerometer and gyroscope
        # Read accelerometer and gyroscope
        readAccGyroData()
        # Magnetometer
        readMagData()

        time.sleep(0.01)
