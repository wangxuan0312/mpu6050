MPU_ADDR=0X68
WHO_AM_I_VAL = MPU_ADDR


MPU_PWR_MGMT1_REG = 0x6B # 电源管理寄存器1
MPU_GYRO_CFG_REG  = 0X1B # 陀螺仪配置寄存器
MPU_ACCEL_CFG_REG = 0X1C # 加速度传感器配置寄存器
MPU_SAMPLE_RATE_REG=0X19 # 设置MPU6050的采样率
MPU_CFG_REG=0X1A # 配置寄存器

MPU_INT_EN_REG=0X38
MPU_USER_CTRL_REG=0X6A
MPU_FIFO_EN_REG=0X23
MPU_INTBP_CFG_REG=0X37
MPU_DEVICE_ID_REG=0X75

MPU_GYRO_XOUTH_REG=0X43
MPU_GYRO_XOUTL_REG=0X44
MPU_GYRO_YOUTH_REG=0X45
MPU_GYRO_YOUTL_REG=0X46
MPU_GYRO_ZOUTH_REG=0X47
MPU_GYRO_ZOUTL_REG=0X48

MPU_ACCEL_XOUTH_REG=0X3B
MPU_ACCEL_XOUTL_REG=0X3C
MPU_ACCEL_YOUTH_REG=0X3D
MPU_ACCEL_YOUTL_REG=0X3E
MPU_ACCEL_ZOUTH_REG=0X3F
MPU_ACCEL_ZOUTL_REG=0X40
MPU_TEMP_OUTH_REG=0X41
MPU_TEMP_OUTL_REG=0X42


config_gyro_range = 3 # 0，±250°/S；1，±500°/S；2，±1000°/S；3，±2000°/S
config_accel_range = 0# 0，±2g；1，±4g；2，±8g；3，±16g

class MPU6050():
    def __init__(self,iicbus,address=WHO_AM_I_VAL):
        self._address = address
        self._bus = iicbus
        self.reset()
    
    def _write_byte(self,cmd,val):
        self._bus.mem_write(val,self._address,cmd)
    def _read_byte(self,cmd):
        buf = self._bus.mem_read(1,self._address,cmd,addr_size=8)
        return int(buf[0])
    
    def reset(self):
        self._write_byte(MPU_PWR_MGMT1_REG, 0x00) # 配置电源管理寄存器 启动MPU6050
        self._write_byte(MPU_GYRO_CFG_REG,  config_gyro_range<<3) # 陀螺仪传感器,±2000dps
        self._write_byte(MPU_ACCEL_CFG_REG, config_accel_range<<3)# 加速度传感器,±2g
        self._write_byte(MPU_SAMPLE_RATE_REG, 0x07) # 采样频率 100
        self._write_byte(MPU_CFG_REG, 0x06) # 设置数字低通滤波器
        self._write_byte(MPU_INT_EN_REG,0X00) #关闭所有中断
        self._write_byte(MPU_USER_CTRL_REG,0X00) #I2C主模式关闭
        self._write_byte(MPU_FIFO_EN_REG,0X00) #关闭FIFO
        self._write_byte(MPU_INTBP_CFG_REG,0X80) #INT引脚低电平有效
    
        buf = self._read_byte(MPU_DEVICE_ID_REG)
        if buf != self._address:
            print("NPU6050 not found!")
        else:
            pass
    
    def _read_byte(self,cmd):
        buf = self._bus.mem_read(1,self._address,cmd,addr_size=8)
        return int(buf[0])
    def _read_u16(self,reg):
        MSB = self._read_byte(reg)
        LSB = self._read_byte(reg)
        return (MSB<< 8) + LSB
    def _read_s16(self,reg):
        result = self._read_u16(reg)
        if result > 32767:result -= 65536
        return result

    def read_Gyro_x(self):
        x = self._read_s16(MPU_GYRO_XOUTH_REG)
        return x
    def read_Gyro_y(self):
        y = self._read_s16(MPU_GYRO_YOUTH_REG)
        return y
    def read_Gyro_z(self):
        z = self._read_s16(MPU_GYRO_ZOUTH_REG)
        return z

    def read_Accel_x(self):
        x = self._read_s16(MPU_ACCEL_XOUTH_REG)
        return x
    def read_Accel_y(self):
        y = self._read_s16(MPU_ACCEL_YOUTH_REG)
        return y
    def read_Accel_z(self):
        z = self._read_s16(MPU_ACCEL_ZOUTH_REG)
        return z

    def read_Temp(self):
        temp = self._read_s16(MPU_TEMP_OUTH_REG)
        return temp


from pyb import I2C

i2c = I2C(2, I2C.MASTER)
mpu = MPU6050(i2c)

def GyroToDegree(num):
    return num / 32768 * 2000

def AccelToGram(num):
    return num / 32768 * 2