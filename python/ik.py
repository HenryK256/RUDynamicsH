import sys, time, math

from RUDynamicsH.python.InverseKinematics import theta

sys.path.append('../lib')
from unitree_actuator_sdk import *

class Motor():
    def __init__(self, id, serial):
        self._serial = serial
        self.cmd = MotorCmd()
        self.data = MotorData()
        self.data.motorType = MotorType.GO_M8010_6
        self.cmd.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        self.cmd.id = id
        self.cmd.dq = 0.0
        self.cmd.kp = 0.0
        self.cmd.kd = 0.01
        self.cmd.tau = 0.0

        self.theta = 0

    def moveToPos(self, pos):
        self.theta=theta
        self.cmd.q = self.theta * queryGearRatio(MotorType.GO_M8010_6) / (2 * math.pi)
        self._serial.sendRecv(self.cmd, self.data)

def generatePath():
    return [0],[0]

def funct(x, y): # Takes in two coordinates => outputs two thetas
    sec1, sec2 = 7.5, 12.9  # Measured lengths of leg segments
    theta0 = math.acos((sec1*sec1+x*x+y*y-sec2*sec2)/(2*sec1*math.sqrt(x*x+y*y))) - math.atan(x/y)
    theta1 = math.acos((sec1*sec1+sec2*sec2-x*x-y*y)/(2*sec1*sec2))
    return theta0, theta1


if __name__ == "__main__":
    serial = SerialPort('/dev/ttyUSB0')

    motor0 = Motor(0, serial)
    motor1 = Motor(1, serial)

    pos_x, pos_y = generatePath()

    for x,y in zip(pos_x, pos_y):
        t1,t2 = funct(x,y)
        motor0.moveToPos(t1)
        motor1.moveToPos(t2)
        time.sleep(0.01)