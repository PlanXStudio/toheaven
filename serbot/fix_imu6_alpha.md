```c
import numpy as np
from threading import Thread
from math import radians

class axis6: 
    STANDARD_GRAVITY = 9.80665
    
    # MPU-6050 Registers
    SMPLRT_DIV   = 0x19 #Sample rate divisor register
    CONFIG       = 0x1A #General configuration register
    GYRO_CONFIG  = 0x1B
    ACCEL_XOUT_H = 0x3B #Accel X_H register
    ACCEL_YOUT_H = 0x3D #Accel Y_H register
    ACCEL_ZOUT_H = 0x3F #Accel Z_H register
    TEMP_OUT_H   = 0x41 
    GYRO_XOUT_H  = 0x43 #Gyro X_H register
    GYRO_YOUT_H  = 0x45 #Gyro Y_H register
    GYRO_ZOUT_H  = 0x47 #Gyro Z_H register   
    PW_MGMT_1    = 0x6B #Primary power/sleep control register

    def __init__(self, bus=1, address=0x68, samplePeriod=0.01, Kp=1.0, Ki=0):
        self.address = address
        """
        if _cat==0 or _cat==1 or _cat==3:
            self.bus = smbus.SMBus(bus)
        elif _cat==4 or _cat==5:
            self.bus = smbus.SMBus(8)
        else:
            del self
        """
        self.bus = smbus.SMBus(1)

        self.bus.write_byte_data(self.address, self.PW_MGMT_1, 1) 
        self.bus.write_byte_data(self.address, self.CONFIG, 0)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 24) 
    
        self.samplePeriod = samplePeriod
        self.Kp = Kp  #proportional gain
        self.Ki = Ki  #integral gain
        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.eInt = [0.0, 0.0, 0.0]

        self.__is_stop = False
        #self.__thread = Thread(target=self.__update)
        #self.__thread.daemon = True;
        #self.__thread.start()

    def __del__(self):
        self.bus.close()

    def __read_reg_data(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
    
        value = ((high << 8) | low)
        
        #to get signed value
        if(value > 32768):
            value = value - 65536

        return value
    
    def __getAccel(self):
        x = self.__read_reg_data(self.ACCEL_XOUT_H)
        y = self.__read_reg_data(self.ACCEL_YOUT_H)
        z = self.__read_reg_data(self.ACCEL_ZOUT_H)

        return (x, y, z)

    def __getGyro(self):
        x = self.__read_reg_data(self.GYRO_XOUT_H)
        y = self.__read_reg_data(self.GYRO_YOUT_H)
        z = self.__read_reg_data(self.GYRO_ZOUT_H)

        return (x, y, z)

    def getTemp(self):
        _t = self.__read_reg_data(self.TEMP_OUT_H)
        return (_t / 340.0) + 36.53
    
    def getAccel(self, axis=None):  #-9.8 ~ 9.8
        _x, _y, _z = self.__getAccel()

        x = (_x / 16384.0) * self.STANDARD_GRAVITY
        y = (_y / 16384.0) * self.STANDARD_GRAVITY
        z = (_z / 16384.0) * self.STANDARD_GRAVITY

        ret = {'x':x, 'y':y, 'z':z} 
        try:
            return ret[axis.lower()]
        except (KeyError, AttributeError):
            return ret

    def getGyro(self, axis=None): #-180.0 ~ 180.0
        _x, _y, _z = self.__getGyro()
        
        x = _x / 131.0
        y = _y / 131.0
        z = _z / 131.0

        ret = {'x':x, 'y':y, 'z':z}
        try:
            return ret[axis.lower()]
        except (KeyError, AttributeError):
            return ret

    def getQuaternion(self):
        return self.quaternion

    def getEuler(self):
        """
        yaw = -np.arctan2(2.0 * (self.quaternion[1] * self.quaternion[2] + self.quaternion[0] * self.quaternion[3]), 
                         pow(self.quaternion[0],2) + pow(self.quaternion[1],2) - pow(self.quaternion[2],2) - pow(self.quaternion[3], 2)) * 57.29577951
        pitch = np.arcsin(2.0 * (self.quaternion[1] * self.quaternion[3] - self.quaternion[0] * self.quaternion[2]) * 57.29577951)
        roll = np.arctan2(2.0 * (self.quaternion[0] * self.quaternion[1] + self.quaternion[2] * self.quaternion[3]), 
                         pow(self.quaternion[0],2) - pow(self.quaternion[1],2) - pow(self.quaternion[2],2) + pow(self.quaternion[3],2)) * 57.29577951
        """
        x, y, z, w = self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3]
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
   
        return (roll, pitch, yaw)

    def update(self):
        #while (not self.__is_stop):
            q1 = self.quaternion[0]
            q2 = self.quaternion[1]
            q3 = self.quaternion[2]
            q4 = self.quaternion[3]

            ax, ay, az = self.__getAccel()
            gx, gy, gz = self.__getGyro()

            gx = np.deg2rad(gx)
            gy = np.deg2rad(gy)
            gz = np.deg2rad(gz)                        

            norm = np.sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0): return # handle NaN

            norm = 1 / norm  # use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            # Estimated direction of gravity
            vx = 2.0 * (q2 * q4 - q1 * q3)
            vy = 2.0 * (q1 * q2 + q3 * q4)
            vz = pow(q1,2) - pow(q2,2) - pow(q3,2) + pow(q4,2)

            # Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy)
            ey = (az * vx - ax * vz)
            ez = (ax * vy - ay * vx)

            if (self.Ki > 0.0):
                self.eInt[0] += ex     # accumulate integral error
                self.eInt[1] += ey
                self.eInt[2] += ez
            else:
                self.eInt[0] = 0.0     # prevent integral wind up
                self.eInt[1] = 0.0
                self.eInt[2] = 0.0

            #/ Apply feedback terms
            gx = gx + self.Kp * ex + self.Ki * self.eInt[0]
            gy = gy + self.Kp * ey + self.Ki * self.eInt[1]
            gz = gz + self.Kp * ez + self.Ki * self.eInt[2]

            # Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5 * self.samplePeriod)
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5 * self.samplePeriod)
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5 * self.samplePeriod)
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5 * self.samplePeriod)

            # Normalise quaternion
            norm = np.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
            norm = 1.0 / norm;
            self.quaternion[0] = q1 * norm
            self.quaternion[1] = q2 * norm
            self.quaternion[2] = q3 * norm
            self.quaternion[3] = q4 * norm

            #time.sleep(self.samplePeriod)

IMU = axis6
```
